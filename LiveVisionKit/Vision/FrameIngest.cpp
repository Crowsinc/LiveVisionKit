#include "FrameIngest.hpp"

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <util/platform.h>
#include <media-io/video-io.h>
#include <media-io/video-frame.h>

/* NOTE:
 *  All upload conversion operations are to YUV, and are performed on the GPU using thread
 *  local cached buffers to maximise performance and avoid expensive GPU memory allocations.
 *
 *  There are a lot of buffers, but only a small subset of them will actually be
 *  allocated because the injest operation is used to convert OBS frames which will
 *  likely have the same video format the entire time.
 *
 *  We prefer YUV over BGR because many vision algorithms require only the Y plane.
 *  Additionally, OBS uses a special color matrix to convert YUV to RGB which makes
 *  OpenCV's YUV to RGB conversion result in different colors than OBS'.
 */

bool operator<<(cv::UMat& dst, const obs_source_frame* src)
{
	return lvk::upload(src, dst);
}

void operator>>(const cv::UMat& src, obs_source_frame* dst)
{
	lvk::download(src, dst);
}

namespace lvk
{

	//-------------------------------------------------------------------------------------

	inline void wrap_data(uint8_t* src, cv::UMat& dst, const uint32_t width, const uint32_t height, const uint32_t line_size, const uint32_t components)
	{
		// NOTE: This is what ultimately uploads OBS plane data to the GPU/CPU UMats
		// and is the bottleneck of the ingest operation. OBS frame planes are actually
		// just pointer offsets to a large contiguous piece of memory starting at the first
		// plane. So it is possible to upload all planes to the GPU/CPU at the same time
		// then extract the other planes through ROIs. In testing this saves around .2 ms
		// when ingesting an 3 plane I420 frame. It is definitely not worth the dangerous
		// dependency on implementation details.

		cv::Mat(height, width, CV_8UC(components), src, line_size).copyTo(dst);
	}

	//-------------------------------------------------------------------------------------

	inline void extract_plane(const obs_source_frame& src, cv::UMat& dst, const uint32_t plane, const float plane_scale_width, const float plane_scale_height,  const uint32_t components)
	{
		wrap_data(
			src.data[plane],
			dst,
			src.width * plane_scale_width,
			src.height * plane_scale_height,
			src.linesize[plane],
			components
		);
	}

	//-------------------------------------------------------------------------------------

	void merge_planes_4xx(const cv::UMat& y, const cv::UMat& u, const cv::UMat& v, cv::UMat& dst)
	{
		thread_local std::vector<cv::UMat> gpu_planes = {
			cv::UMat(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
			cv::UMat(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
			cv::UMat(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY)
		};

		// All planar 4xx formats have a full size Y plane, with potentially
		// sub-sampled U and V planes. So use Y directly, resize U and V to match
		// the size of Y then merge to obtain a packed YUV frame.

		const cv::Size frame_size = y.size();

		gpu_planes[0] = y;
		cv::resize(u, gpu_planes[1], frame_size, 0, 0, cv::INTER_NEAREST);
		cv::resize(v, gpu_planes[2], frame_size, 0, 0, cv::INTER_NEAREST);

		cv::merge(gpu_planes, dst);
	}

	//-------------------------------------------------------------------------------------

	void upload_planar_4xx(const obs_source_frame& src, cv::UMat& dst, const float chroma_scale_x, const float chroma_scale_y)
	{
		thread_local cv::UMat gpu_plane_y(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat gpu_plane_u(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat gpu_plane_v(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		// All planar 4xx formats have a full size Y plane, with potentially
		// sub-sampled U and V planes. So extract all planes then merge them into
		// a full YUV frame.

		extract_plane(src, gpu_plane_y, 0, 1.0f, 1.0f, 1);
		extract_plane(src, gpu_plane_u, 1, chroma_scale_x, chroma_scale_y, 1);
		extract_plane(src, gpu_plane_v, 2, chroma_scale_x, chroma_scale_y, 1);

		merge_planes_4xx(gpu_plane_y, gpu_plane_u, gpu_plane_v, dst);
	}

	//-------------------------------------------------------------------------------------

	void upload_semi_planar_nv12(const obs_source_frame& src, cv::UMat& dst)
	{
		thread_local cv::UMat gpu_plane_y(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat gpu_plane_u(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat gpu_plane_v(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat gpu_plane_uv(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		// Semi-planar NV12 contains a full Y plane, and a packed plane of interleaved U and V.
		// OpenCV provides 'cvtColorTwoPlane' for directly converting NVXX, but the function
		// takes around ~4ms to run (January 2022). So we instead extract the U and V planes
		// from the second plane, and treat it as planar 420 for a 4x speed up.

		extract_plane(src, gpu_plane_y, 0, 1.0f, 1.0f, 1);
		extract_plane(src, gpu_plane_uv, 1, 0.5f, 0.5f, 2);

		cv::extractChannel(gpu_plane_uv, gpu_plane_u, 0);
		cv::extractChannel(gpu_plane_uv, gpu_plane_v, 1);

		merge_planes_4xx(gpu_plane_y, gpu_plane_u, gpu_plane_v, dst);
	}

	//-------------------------------------------------------------------------------------

	void upload_packed_422(const obs_source_frame& src, cv::UMat& dst, const bool y_first, const bool u_first)
	{
		thread_local cv::UMat gpu_buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		// Packed 422 contains interleaved sets of YU and YV for every two pixels. It is equivalent to
		// a two component image where the first component is Y and second component is interleaved
		// U and V; similar to NV12 but in a single plane and with only horizontal sub-sampling.
		// We can use OpenCV to directly convert it to BGR, and then convert it back to YUV/

		const auto bgr_conversion = y_first ?
				(u_first ? cv::COLOR_YUV2BGR_YUYV : cv::COLOR_YUV2BGR_YVYU) :
				(u_first ? cv::COLOR_YUV2BGR_UYVY : -1 /* VYUY UNSUPPORTED */);

		extract_plane(src, gpu_buffer, 0, 1.0f, 1.0f, 2);

		cv::cvtColor(gpu_buffer, dst, bgr_conversion);
		cv::cvtColor(dst, dst, cv::COLOR_BGR2YUV);
	}

	//-------------------------------------------------------------------------------------

	void upload_packed_direct(const obs_source_frame& src, cv::UMat& dst, const uint32_t components, const cv::ColorConversionCodes conversion)
	{
		thread_local cv::UMat gpu_buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		// Simple packed uncompressed formats like RGBA don't need to be processed, hence
		// can be directly wrapped into a buffer, and converted to the required color format.
		wrap_data(
			src.data[0],
			dst,
			src.width,
			src.height,
			src.linesize[0],
			components
		);
		cv::cvtColor(gpu_buffer, dst, conversion);
	}

	//-------------------------------------------------------------------------------------

	void upload_packed_direct(const obs_source_frame& src, cv::UMat& dst, const uint32_t components, const cv::ColorConversionCodes conversion_1, const cv::ColorConversionCodes conversion_2)
	{
		thread_local cv::UMat gpu_buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		upload_packed_direct(src, gpu_buffer, components, conversion_1);
		cv::cvtColor(gpu_buffer, dst, conversion_2);
	}

	//-------------------------------------------------------------------------------------

	bool upload(const obs_source_frame* src, cv::UMat& dst)
	{
		const auto& frame = *src;

		switch(frame.format)
		{
			// Planar 4xx formats
			case video_format::VIDEO_FORMAT_YUVA:
			case video_format::VIDEO_FORMAT_I444:
				upload_planar_4xx(frame, dst, 1.0f, 1.0f);
				break;
			case video_format::VIDEO_FORMAT_I42A:
			case video_format::VIDEO_FORMAT_I422:
				upload_planar_4xx(frame, dst, 0.5f, 1.0f);
				break;
			case video_format::VIDEO_FORMAT_I40A:
			case video_format::VIDEO_FORMAT_I420:
				upload_planar_4xx(frame, dst, 0.5f, 0.5f);
				break;

			// Semi-planar NV12 format
			case video_format::VIDEO_FORMAT_NV12:
				upload_semi_planar_nv12(frame, dst);
				break;

			// Packed 42x formats
			case video_format::VIDEO_FORMAT_YVYU:
				upload_packed_422(frame, dst, true, false);
				break;
			case video_format::VIDEO_FORMAT_YUY2:
				upload_packed_422(frame, dst, true, true);
				break;
			case video_format::VIDEO_FORMAT_UYVY:
				upload_packed_422(frame, dst, false, true);
				break;

			// Packed uncompressed non-YUV formats
			case video_format::VIDEO_FORMAT_Y800:
				upload_packed_direct(frame, dst, 1, cv::COLOR_GRAY2BGR, cv::COLOR_BGR2YUV);
				break;
			case video_format::VIDEO_FORMAT_RGBA:
				upload_packed_direct(frame, dst, 4, cv::COLOR_RGBA2RGB, cv::COLOR_RGB2YUV);
				break;
			case video_format::VIDEO_FORMAT_BGRX:
			case video_format::VIDEO_FORMAT_BGRA:
				upload_packed_direct(frame, dst, 4, cv::COLOR_BGRA2BGR, cv::COLOR_BGR2YUV);
				break;
			case video_format::VIDEO_FORMAT_BGR3:
				upload_packed_direct(frame, dst, 3, cv::COLOR_BGR2YUV);
				break;

			// Unsupported formats
			default:
				return false;
		}
		return true;
	}

	//-------------------------------------------------------------------------------------

	void download(const cv::UMat& src, obs_source_frame* dst)
	{
		const auto& frame = *dst;

		switch(frame.format)
		{





		}
	}

	//-------------------------------------------------------------------------------------
}

