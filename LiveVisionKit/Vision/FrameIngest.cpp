#include "FrameIngest.hpp"

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

/* NOTE:
 *  All ingest operations are performed on the GPU using thread local cached buffers
 *  in order to maximise performance and avoid expensive GPU memory allocations.
 *
 *  There are a lot of buffers, but only a small subset of them will actually be
 *  allocated because the injest operation is used to convert OBS frames which will
 *  likely have the same video format the entire time.
 */

bool operator<<(cv::UMat& dst, const obs_source_frame& frame)
{
	return lvk::ingest(frame, dst);
}

namespace lvk
{
	//-------------------------------------------------------------------------------------

	void wrap_plane(const obs_source_frame& frame, cv::UMat& dst, const uint32_t plane, const cv::Size size, const int type)
	{
		// NOTE: Turns out wrapping the data in a normal Mat and copying it to the UMat is the fastest
		// way to upload the data to the GPU. Host UMat to Device UMat copying is much faster, but getting
		// the Host UMat in the first place is too slow.
		cv::Mat(size, type, frame.data[plane], frame.linesize[plane]).copyTo(dst);
	}

	//-------------------------------------------------------------------------------------

	void extract_plane(const obs_source_frame& frame, cv::UMat& dst, const uint32_t plane, const float scale_x, const float scale_y, const int type)
	{
		wrap_plane(
			frame,
			dst,
			plane,
			cv::Size(frame.width * scale_x, frame.height * scale_y),
			type
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

		// All planar 4xx formats have Y as full size, with potentially
		// sub-sampled U and V so use Y directly, resize u and v, then
		// perform color converion.

		const cv::Size frame_size = y.size();

		gpu_planes[0] = y;
		cv::resize(u, gpu_planes[1], frame_size, 0, 0, cv::INTER_LINEAR);
		cv::resize(v, gpu_planes[2], frame_size, 0, 0, cv::INTER_LINEAR);

		cv::merge(gpu_planes, dst);

		// Can perform this in place as YUV and BGR use the same sized mat.
		cv::cvtColor(dst, dst, cv::COLOR_YUV2BGR);
	}

	//-------------------------------------------------------------------------------------

	void ingest_planar_4xx(const obs_source_frame& frame, cv::UMat& dst, const float chroma_scale_x, const float chroma_scale_y)
	{
		thread_local cv::UMat gpu_plane_y(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat gpu_plane_u(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat gpu_plane_v(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		// Planar 4xx contains a full Y plane, and potentially sub-sampled U and V
		// chroma channels in separate planes. We extract all three planes, upsampling
		// the U and V planes using linear interpolation. All planes are then merged
		// and converted from YUV to the desired color format.
		extract_plane(frame, gpu_plane_y, 0, 1.0f, 1.0f, CV_8UC1);
		extract_plane(frame, gpu_plane_u, 1, chroma_scale_x, chroma_scale_y, CV_8UC1);
		extract_plane(frame, gpu_plane_v, 2, chroma_scale_x, chroma_scale_y, CV_8UC1);

		merge_planes_4xx(gpu_plane_y, gpu_plane_u, gpu_plane_v, dst);
	}

	//-------------------------------------------------------------------------------------

	void ingest_semi_planar_nv12(const obs_source_frame& frame, cv::UMat& dst)
	{
		thread_local cv::UMat gpu_plane_y(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat gpu_plane_u(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat gpu_plane_v(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat gpu_buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		// Semi-planar NV12 contains a full Y plane, and a set of half sub-sampled U and V
		// channels packed into a full second plane. OpenCV provides 'cvtColorTwoPlane' for
		// converting NVxx, but the function is too slow (~4ms). So we instead extract the
		// U and V planes from the second plane, and treat it as planar 420 (~1ms total).

		extract_plane(frame, gpu_plane_y, 0, 1.0f, 1.0f, CV_8UC1);
		extract_plane(frame, gpu_buffer, 1, 0.5f, 0.5f, CV_8UC2);

		cv::extractChannel(gpu_buffer, gpu_plane_u, 0);
		cv::extractChannel(gpu_buffer, gpu_plane_v, 1);

		merge_planes_4xx(gpu_plane_y, gpu_plane_u, gpu_plane_v, dst);
	}

	//-------------------------------------------------------------------------------------

	void ingest_packed_42x(const obs_source_frame& frame, cv::UMat& dst, const float chroma_scale_x, const cv::ColorConversionCodes code)
	{
		thread_local cv::UMat gpu_buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		// Packed 4xx contains a single plane of interleaved Y, and potentially sub-sampled U, and V components in
		// the horizontal direction. Since all components are packed together, the width of the frame is scaled
		// to include all the required U and V components for each row. The packed plane follows the format used
		// by OpenCV Mats, so we can directly use the built in converters to change the color format.
		extract_plane(frame,gpu_buffer, 0, 1.0f + (2.0f * chroma_scale_x), 1.0f, CV_8UC1);
		cv::cvtColor(gpu_buffer, dst, code);
	}

	//-------------------------------------------------------------------------------------

	void ingest_packed_direct(const obs_source_frame& frame, cv::UMat& dst, const uint32_t components, const cv::ColorConversionCodes code = cv::COLOR_COLORCVT_MAX)
	{
		thread_local cv::UMat gpu_buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		// Simple packed uncompressed formats like RGBA don't need to be processed, hence
		// can be directly input into a buffer, and converted to the required color format.
		// If no color format code is provided, then just copy the plane over.

		const bool skip_conversion = (code == cv::COLOR_COLORCVT_MAX);

		wrap_plane(
			frame,
			skip_conversion ? dst : gpu_buffer,
			0,
			cv::Size(frame.width,frame.height),
			CV_8UC(components)
		);

		if(!skip_conversion)
			cv::cvtColor(gpu_buffer, dst, code);
	}

	//-------------------------------------------------------------------------------------

	bool ingest(const obs_source_frame& frame, cv::UMat& dst)
	{
		switch(frame.format)
		{
			// Planar 4xx formats
			// NOTE: ignore the alpha plane if it exists
			case video_format::VIDEO_FORMAT_YUVA:
			case video_format::VIDEO_FORMAT_I444:
				ingest_planar_4xx(frame, dst, 1.0f, 1.0f);
				break;

			case video_format::VIDEO_FORMAT_I42A:
			case video_format::VIDEO_FORMAT_I422:
				ingest_planar_4xx(frame, dst, 0.5f, 1.0f);
				break;
			case video_format::VIDEO_FORMAT_I40A:
			case video_format::VIDEO_FORMAT_I420:
				ingest_planar_4xx(frame, dst, 0.5f, 0.5f);
				break;

			// Semi-planar NV12 format
			case video_format::VIDEO_FORMAT_NV12:
				ingest_semi_planar_nv12(frame, dst);
				break;

			// Packed 42x formats
			//TODO: test all these formats
			case video_format::VIDEO_FORMAT_YVYU:
				ingest_packed_42x(frame, dst, 0.5f, cv::COLOR_YUV2BGR_YVYU);
				break;
			case video_format::VIDEO_FORMAT_YUY2:
				ingest_packed_42x(frame, dst, 0.5f, cv::COLOR_YUV2BGR_YUYV);
				break;
			case video_format::VIDEO_FORMAT_UYVY:
				ingest_packed_42x(frame, dst, 0.5f, cv::COLOR_YUV2BGR_UYVY);
				break;

			// Packed uncompressed formats
			case video_format::VIDEO_FORMAT_Y800:
				ingest_packed_direct(frame, dst, 1, cv::COLOR_GRAY2BGR);
				break;
			case video_format::VIDEO_FORMAT_RGBA:
				ingest_packed_direct(frame, dst, 4, cv::COLOR_RGBA2BGR);
				break;
			case video_format::VIDEO_FORMAT_BGRX:
			case video_format::VIDEO_FORMAT_BGRA:
				ingest_packed_direct(frame, dst, 4, cv::COLOR_BGRA2BGR);
				break;
			case video_format::VIDEO_FORMAT_BGR3:
				ingest_packed_direct(frame, dst, 3);
				break;

			// NOTE: AYUV format is not currently supported
			default:
				return false;
		}
		return true;
	}

}
