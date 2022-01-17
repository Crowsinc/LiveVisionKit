#include "FrameIngest.hpp"

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <functional>
#include <util/platform.h>

/* NOTE:
 *  All ingest operations are performed on the GPU using thread local cached buffers
 *  in order to maximise performance and avoid expensive GPU memory allocations.
 *
 *  Only a small subset of the buffers will actually be allocated because
 *  the injest operation is used to convert OBS frames which will likely have
 *  the same video format for the entire LVK run time.
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

	void extract_plane(const obs_source_frame& frame, cv::UMat& dst, const uint32_t plane, const float scale_x, const float scale_y)
	{
		wrap_plane(
			frame,
			dst,
			plane,
			cv::Size(frame.width * scale_x, frame.height * scale_y),
			CV_8UC1
		);
	}

	//-------------------------------------------------------------------------------------

	void ingest_planar_4xx(const obs_source_frame& frame, cv::UMat& dst, const float chroma_scale_x, const float chroma_scale_y, const cv::ColorConversionCodes code)
	{
		thread_local std::vector<cv::UMat> gpu_planes = {
			cv::UMat(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
			cv::UMat(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
			cv::UMat(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY)
		};
		thread_local cv::UMat gpu_buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		// We use a separate chroma buffer to avoid unnecessary GPU allocations because
		// the chroma planes are likely sub-sampled so require a different sized buffer.
		thread_local cv::UMat gpu_chroma_buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		// Planar 4xx contains a full Y plane, and potentially sub-sampled U and V
		// chroma channels in separate planes. We extract all three planes, upsampling
		// the U and V planes using linear interpolation. All planes are then merged
		// and converted from YUV to the desired color format.
		extract_plane(frame, gpu_planes[0], 0, 1.0f, 1.0f);

		extract_plane(frame, gpu_chroma_buffer, 1, chroma_scale_x, chroma_scale_y);
		cv::resize(gpu_chroma_buffer, gpu_planes[1], cv::Size(frame.width, frame.height), 0, 0, cv::INTER_LINEAR);

		extract_plane(frame, gpu_chroma_buffer, 2, chroma_scale_x, chroma_scale_y);
		cv::resize(gpu_chroma_buffer, gpu_planes[2], cv::Size(frame.width, frame.height), 0, 0, cv::INTER_LINEAR);

		cv::merge(gpu_planes, gpu_buffer);

		cv::cvtColor(gpu_buffer, dst, code);
	}


	//-------------------------------------------------------------------------------------

	void ingest_packed_42x(const obs_source_frame& frame, cv::UMat& dst, const float chroma_scale_x, const cv::ColorConversionCodes code)
	{
		thread_local cv::UMat gpu_buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		// Packed 4xx contains a single plane of interleaved Y, and potentially sub-sampled U, and V components in
		// the horizontal direction. Since all components are packed together, the width of the frame is scaled
		// to include all the required U and V components for each row. The packed plane follows the format used
		// by OpenCV Mats, so we can directly use the built in converters to change the color format.
		extract_plane(frame,gpu_buffer, 0, 1.0f + (2.0f * chroma_scale_x), 1.0f);
		cv::cvtColor(gpu_buffer, dst, code);
	}

	//-------------------------------------------------------------------------------------

	void ingest_semi_planar_nvxx(const obs_source_frame& frame, cv::UMat& dst, const cv::ColorConversionCodes code)
	{
		thread_local cv::UMat gpu_plane_0(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat gpu_plane_1(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat gpu_buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		// Semi-planar NVxx contains a full Y plane, and a set of half sub-sampled U and V
		// channels packed into a full second plane. We extract the two planes then use
		// OpenCV to convert the color format.
		extract_plane(frame, gpu_plane_0, 0, 1.0f, 1.0f);
		extract_plane(frame, gpu_plane_1, 1, 1.0f, 1.0f);

		cv::cvtColorTwoPlane(gpu_plane_0, gpu_plane_1, dst, code);
	}

	//-------------------------------------------------------------------------------------

	void ingest_packed_direct(const obs_source_frame& frame, cv::UMat& dst, const uint32_t components, const cv::ColorConversionCodes code = cv::COLOR_COLORCVT_MAX)
	{
		thread_local cv::UMat gpu_buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		// Simple packed uncompressed formats like RGBA don't need to be processed, hence
		// can be directly input into a buffer, and converted to the required color format.
		// If no color format code is provided, then just copy the plane over.

		const bool skip_conversion = code == cv::COLOR_COLORCVT_MAX;

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
			case video_format::VIDEO_FORMAT_I444:
				ingest_planar_4xx(frame, dst, 1.0f, 1.0f, cv::COLOR_YUV2BGR);
				break;
			case video_format::VIDEO_FORMAT_I422:
				ingest_planar_4xx(frame, dst, 0.5f, 1.0f, cv::COLOR_YUV2BGR);
				break;
			case video_format::VIDEO_FORMAT_I420:
				ingest_planar_4xx(frame, dst, 0.5f, 0.5f, cv::COLOR_YUV2BGR);
				break;

			// Packed 42x formats
			case video_format::VIDEO_FORMAT_YVYU:
				ingest_packed_42x(frame, dst, 0.5f, cv::COLOR_YUV2BGR_YVYU);
				break;
			case video_format::VIDEO_FORMAT_YUY2:
				ingest_packed_42x(frame, dst, 0.5f, cv::COLOR_YUV2BGR_YUYV);
				break;
			case video_format::VIDEO_FORMAT_UYVY:
				ingest_packed_42x(frame, dst, 0.5f, cv::COLOR_YUV2BGR_UYVY);
				break;

			// Semi-planar NVxx formats
			case video_format::VIDEO_FORMAT_NV12:
				ingest_semi_planar_nvxx(frame, dst, cv::COLOR_YUV2BGR_NV12);
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

			// NOTE: All YUVA formats are currently unsupported.
			default:
				return false;
		}
		return true;
	}

}
