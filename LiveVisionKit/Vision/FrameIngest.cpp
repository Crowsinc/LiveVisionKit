#include "FrameIngest.hpp"


/* NOTE:
 *  All upload conversion operations are to YUV, and are performed on the GPU using thread
 *  local cached buffers to maximise performance and avoid expensive GPU memory allocations.
 *  Thread local is used just in case the API is ever accessed via multiple OBS threads at some point.
 *
 *  We assume that the format being upload/downloaded is unlikely to change during a stream
 *  so only a subset of the cached buffers will be utilised, without needing to be resized
 *  or re-allocated between downloads/uploads. There are a lot of buffers involved but the
 *  GPU memory is less important than minimising the overhead of the OBS frame to OpenCV mat
 *  conversion, which is an unproductive but necessary operation.
 *
 *  We prefer YUV over BGR because many vision algorithms require only the Y plane.
 *  Additionally, OBS uses a special color matrix to convert YUV to RGB which makes
 *  OpenCV's YUV to RGB conversion result in different colors than OBS'.
 */

bool operator<<(cv::UMat& dst, const obs_source_frame* src)
{
	return lvk::extract_frame(src, dst);
}

void operator>>(const cv::UMat& src, obs_source_frame* dst)
{
	lvk::insert_frame(src, dst);
}

namespace lvk
{

	//-------------------------------------------------------------------------------------

	inline void fill_plane(obs_source_frame& dst, const uint32_t plane, const uint8_t value)
	{
		std::memset(dst.data[plane], value, dst.width * dst.height);
	}

	//-------------------------------------------------------------------------------------

	inline void merge_planes(const cv::UMat& p1, const cv::UMat& p2, const cv::UMat& p3, cv::UMat& dst)
	{
		thread_local std::vector<cv::UMat> planes(3);

		planes[0] = p1;
		planes[1] = p2;
		planes[2] = p3;

		cv::merge(planes, dst);
	}

	//-------------------------------------------------------------------------------------

	inline void merge_planes(const cv::UMat& p1, const cv::UMat& p2, cv::UMat& dst)
	{
		thread_local std::vector<cv::UMat> planes(2);

		planes[0] = p1;
		planes[1] = p2;

		cv::merge(planes, dst);
	}

	//-------------------------------------------------------------------------------------

	inline void split_planes(const cv::UMat& src, cv::UMat& p1, cv::UMat& p2, cv::UMat& p3)
	{
		thread_local std::vector<cv::UMat> planes(3);

		cv::split(src, planes);

		p1 = planes[0];
		p2 = planes[1];
		p3 = planes[2];
	}

	//-------------------------------------------------------------------------------------

	inline void split_planes(const cv::UMat& src, cv::UMat& p1, cv::UMat& p2)
	{
		thread_local std::vector<cv::UMat> planes(2);

		cv::split(src, planes);

		p1 = planes[0];
		p2 = planes[1];
	}

	//-------------------------------------------------------------------------------------

	inline void extract_data(uint8_t* src, cv::UMat& dst, const uint32_t width, const uint32_t height, const uint32_t line_size, const uint32_t components)
	{
		// NOTE: This is what ultimately uploads OBS plane data to the GPU/CPU UMats
		// and is the bottleneck of the ingest operation. OBS frame planes are actually
		// just pointer offsets to a large contiguous piece of memory starting at the first
		// plane. So it is possible to upload all planes to the GPU/CPU at the same time
		// then extract the other planes through ROIs. In testing this saves around .2 ms
		// when ingesting a 3 plane I420 frame. It is definitely not worth the dependency
		// on implementation details.
		cv::Mat(height, width, CV_8UC(components), src, line_size).copyTo(dst);
	}

	//-------------------------------------------------------------------------------------

	inline void insert_data(const cv::UMat& src, uint8_t* dst)
	{
		// Wrap the destination data in a Mat header and perform the download using an optimised OpenCV copy.
		// This assumes that the destination is large enough to actually hold all the src data.
		src.copyTo(cv::Mat(src.size(), src.type(), dst));
	}

	//-------------------------------------------------------------------------------------

	inline void extract_plane(const obs_source_frame& src, cv::UMat& dst, const uint32_t plane, const float width_scaling, const float height_scaling,  const uint32_t components)
	{
		extract_data(
			src.data[plane],
			dst,
			src.width * width_scaling,
			src.height * height_scaling,
			src.linesize[plane],
			components
		);
	}

	//-------------------------------------------------------------------------------------

	inline void insert_plane(const cv::UMat& src, obs_source_frame& dst, const uint32_t plane)
	{
		insert_data(src, dst.data[plane]);
		dst.linesize[plane] = src.step;
	}

	//-------------------------------------------------------------------------------------

	void extract_planar_4xx(const obs_source_frame& src, cv::UMat& dst, const bool subsampled_width, const bool subsampled_height)
	{
		thread_local cv::UMat buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_y(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_u(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_v(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		// All planar 4xx formats have a full size Y plane, with potentially
		// sub-sampled U and V planes. So extract all planes then merge them into
		// a full packed YUV frame.

		extract_plane(src, plane_y, 0, 1.0f, 1.0f, 1);

		if(subsampled_width || subsampled_height)
		{
			const float chroma_width_scale = subsampled_width ? 0.5 : 1.0;
			const float chroma_height_scale = subsampled_height ? 0.5 : 1.0;

			extract_plane(src, buffer, 1, chroma_width_scale, chroma_height_scale, 1);
			cv::resize(buffer, plane_u, plane_y.size(), 0, 0, cv::INTER_NEAREST);

			extract_plane(src, buffer, 2, chroma_width_scale, chroma_height_scale, 1);
			cv::resize(buffer, plane_v, plane_y.size(), 0, 0, cv::INTER_NEAREST);
		}
		else
		{
			extract_plane(src, plane_u, 1, 1.0, 1.0, 1);
			extract_plane(src, plane_v, 2, 1.0, 1.0, 1);
		}

		merge_planes(plane_y, plane_u, plane_v, dst);
	}

	//-------------------------------------------------------------------------------------

	void extract_semi_planar_nv12(const obs_source_frame& src, cv::UMat& dst)
	{
		thread_local cv::UMat buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_y(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_uv(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		// Semi-planar NV12 contains a full Y plane, and a packed plane of 4:2:0 subsampled U and V.
		// OpenCV provides 'cvtColorTwoPlane' for directly converting NVXX, but the function
		// takes around ~4ms to run (January 2022). So we instead extract the packed UV plane,
		// resize it to remove subsampling and then mix them with the Y plane to end up with a
		// packed YUV frame as required.

		extract_plane(src, plane_y, 0, 1.0, 1.0, 1);

		extract_plane(src, buffer, 1, 0.5, 0.5, 2);
		cv::resize(buffer, plane_uv, plane_y.size(), 0, 0, cv::INTER_NEAREST);

		// Must be pre-allocated for the mixChannels function (doesn't unnecessarily re-allocate).
		dst.create(plane_y.size(), CV_8UC3, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		// NOTE: need to explicitly set destination as a UMat vector to invoke OpenCL optimisations.
		cv::mixChannels({{plane_y, plane_uv}}, std::vector<cv::UMat>(1, dst), {0,0,  1,1,  2,2});
	}

	//-------------------------------------------------------------------------------------

	void extract_packed_422(const obs_source_frame& src, cv::UMat& dst, const bool y_first, const bool u_first)
	{
		thread_local cv::UMat buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		// Packed 422 contains interleaved sets of YU and YV for every two pixels. It is equivalent to
		// a two component image where the first component is Y and second component is interleaved
		// U and V; similar to NV12 but in a single plane and with only horizontal sub-sampling.
		// We can use OpenCV to directly convert it to BGR, and then convert it back to YUV

		const auto bgr_conversion = y_first ?
				(u_first ? cv::COLOR_YUV2BGR_YUYV : cv::COLOR_YUV2BGR_YVYU) :
				(u_first ? cv::COLOR_YUV2BGR_UYVY : -1 /* VYUY UNSUPPORTED */);

		extract_plane(src, buffer, 0, 1.0, 1.0, 2);

		cv::cvtColor(buffer, dst, bgr_conversion);
		cv::cvtColor(dst, dst, cv::COLOR_BGR2YUV);
	}

	//-------------------------------------------------------------------------------------

	void extract_packed_direct_stepped(const obs_source_frame& src, cv::UMat& dst, const uint32_t components, const cv::ColorConversionCodes conversion_1, const cv::ColorConversionCodes conversion_2)
	{
		thread_local cv::UMat buffer_1(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat buffer_2(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		// Simple packed uncompressed formats like RGBA don't need to be processed, hence
		// can be directly wrapped into a buffer, and converted to the required color format.
		extract_data(
			src.data[0],
			buffer_1,
			src.width,
			src.height,
			src.linesize[0],
			components
		);

		cv::cvtColor(buffer_1, buffer_2, conversion_1);
		cv::cvtColor(buffer_2, dst, conversion_2);
	}

	//-------------------------------------------------------------------------------------

	void extract_packed_direct(const obs_source_frame& src, cv::UMat& dst, const uint32_t components, const cv::ColorConversionCodes conversion)
	{
		thread_local cv::UMat buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		// Simple packed uncompressed formats like RGBA don't need to be processed, hence
		// can be directly wrapped into a buffer, and converted to the required color format.
		extract_data(
			src.data[0],
			buffer,
			src.width,
			src.height,
			src.linesize[0],
			components
		);

		cv::cvtColor(buffer, dst, conversion);
	}


	//-------------------------------------------------------------------------------------

	bool extract_frame(const obs_source_frame* src, cv::UMat& dst)
	{
		const auto& frame = *src;

		switch(frame.format)
		{
			// Planar 4xx formats
			case video_format::VIDEO_FORMAT_YUVA:
			case video_format::VIDEO_FORMAT_I444:
				extract_planar_4xx(frame, dst, false, false);
				break;
			case video_format::VIDEO_FORMAT_I42A:
			case video_format::VIDEO_FORMAT_I422:
				extract_planar_4xx(frame, dst, true, false);
				break;
			case video_format::VIDEO_FORMAT_I40A:
			case video_format::VIDEO_FORMAT_I420:
				extract_planar_4xx(frame, dst, true, true);
				break;

			// Semi-planar NV12 format
			case video_format::VIDEO_FORMAT_NV12:
				extract_semi_planar_nv12(frame, dst);
				break;

			// Packed 42x formats
			case video_format::VIDEO_FORMAT_YVYU:
				extract_packed_422(frame, dst, true, false);
				break;
			case video_format::VIDEO_FORMAT_YUY2:
				extract_packed_422(frame, dst, true, true);
				break;
			case video_format::VIDEO_FORMAT_UYVY:
				extract_packed_422(frame, dst, false, true);
				break;

			// Packed uncompressed non-YUV formats
			case video_format::VIDEO_FORMAT_Y800:
				extract_packed_direct_stepped(frame, dst, 1, cv::COLOR_GRAY2BGR, cv::COLOR_BGR2YUV);
				break;
			case video_format::VIDEO_FORMAT_RGBA:
				extract_packed_direct_stepped(frame, dst, 4, cv::COLOR_RGBA2RGB, cv::COLOR_RGB2YUV);
				break;
			case video_format::VIDEO_FORMAT_BGRX:
			case video_format::VIDEO_FORMAT_BGRA:
				extract_packed_direct_stepped(frame, dst, 4, cv::COLOR_BGRA2BGR, cv::COLOR_BGR2YUV);
				break;
			case video_format::VIDEO_FORMAT_BGR3:
				extract_packed_direct(frame, dst, 3, cv::COLOR_BGR2YUV);
				break;

			// Unsupported formats
			default:
				return false;
		}
		return true;
	}


	//-------------------------------------------------------------------------------------

	void insert_planar_4xx(const cv::UMat& src, obs_source_frame& dst, const bool subsample_width, const bool subsample_height)
	{
		thread_local cv::UMat buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_y(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_u(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_v(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		// Planar 4xx consists of a full Y plane and potentially subsampled U and V planes.
		// So split the packed source into planes, insert the Y plane, and subsample the
		// U and V planes before also inserting them.

		split_planes(src, plane_y, plane_u, plane_v);

		insert_plane(plane_y, dst, 0);

		if(subsample_width || subsample_height)
		{
			const auto chroma_width_scale = subsample_width ? 0.5 : 1.0;
			const auto chroma_height_scale = subsample_height ? 0.5 : 1.0;

			cv::resize(plane_u, buffer, cv::Size(), chroma_width_scale, chroma_height_scale, cv::INTER_NEAREST);
			insert_plane(buffer, dst, 1);

			cv::resize(plane_v, buffer, cv::Size(), chroma_width_scale, chroma_height_scale, cv::INTER_NEAREST);
			insert_plane(buffer, dst, 2);
		}
		else
		{
			insert_plane(plane_u, dst, 1);
			insert_plane(plane_v, dst, 2);
		}
	}

	//-------------------------------------------------------------------------------------

	void insert_semi_planar_nv12(const cv::UMat& src, obs_source_frame& dst)
	{
		thread_local cv::UMat buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_y(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_uv(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		// Semi-planar NV12 consists of a full Y plane and a packed subsampled U and V plane.
		// So simply insert the Y plane. Remove the Y plane from the source, and subsample
		// the resulting packed U and V plane before inserting it directly into the frame.

		cv::extractChannel(src, plane_y, 0);
		insert_plane(plane_y, dst, 0);

		// Must be pre-allocated for the mixChannels function (doesn't unnecessarily re-allocate).
		buffer.create(src.size(), CV_8UC2, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		// NOTE: need to explicitly set destination as a UMat vector to invoke OpenCL optimisations.
		cv::mixChannels({src}, std::vector<cv::UMat>{buffer}, {1,0,  2,1});
		cv::resize(buffer, plane_uv, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);

		insert_plane(plane_uv, dst, 1);
	}

	//-------------------------------------------------------------------------------------

	void insert_packed_422(const cv::UMat& src, obs_source_frame& dst, const bool y_first, const bool u_first)
	{
		thread_local cv::UMat buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_y(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_uv(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		// Packed 422 contains interleaved sets of YU and YV for every two pixels. It is equivalent to
		// a two component image where the first component is Y and second component is interleaved
		// U and V. So we can take the packed U and V components of the source, subsample them horizontally
		// then re-interpret them as a single component plane consisting of interleaved U and V parts.
		// The interleaved UV plane can then be inserted back in with the Y plane resulting in a packed 422
		// format that can be inserted into the frame.

		cv::extractChannel(src, plane_y, 0);

		// Must be pre-allocated for the mixChannels function (doesn't unnecessarily re-allocate).
		buffer.create(src.size(), CV_8UC2, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		if(u_first)
			cv::mixChannels({src}, std::vector<cv::UMat>{buffer}, {1,0,  2,1});
		else
			cv::mixChannels({src}, std::vector<cv::UMat>{buffer}, {2,0,  1,1});

		cv::resize(buffer, plane_uv, cv::Size(), 0.5, 1.0, cv::INTER_NEAREST);
		plane_uv = plane_uv.reshape(1, plane_uv.rows);

		if(y_first)
			cv::mixChannels({{plane_y, plane_uv}}, std::vector<cv::UMat>{buffer}, {0,0,  1,1});
		else
			cv::mixChannels({{plane_y, plane_uv}}, std::vector<cv::UMat>{buffer}, {0,1,  1,0});

		insert_plane(buffer, dst, 0);
	}

	//-------------------------------------------------------------------------------------

	void insert_packed_direct_stepped(const cv::UMat& src, obs_source_frame& dst, const cv::ColorConversionCodes conversion_1, const cv::ColorConversionCodes conversion_2)
	{
		thread_local cv::UMat buffer_1(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat buffer_2(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		// For a simple uncompressed format we just need to perform the color
		// conversion and insert the entire packed plane into the frame.

		cv::cvtColor(src, buffer_1, conversion_1);
		cv::cvtColor(buffer_1, buffer_2, conversion_2);
		insert_plane(buffer_2, dst, 0);
	}

	//-------------------------------------------------------------------------------------

	void insert_packed_direct(const cv::UMat& src, obs_source_frame& dst, const cv::ColorConversionCodes conversion)
	{
		thread_local cv::UMat buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		// For a simple uncompressed format we just need to perform the color
		// conversion and insert the entire packed plane into the frame.

		cv::cvtColor(src, buffer, conversion);
		insert_plane(buffer, dst, 0);
	}

	//-------------------------------------------------------------------------------------

	void insert_frame(const cv::UMat& src, obs_source_frame* dst)
	{
		auto& frame = *dst;

		switch(frame.format)
		{
			// Planar 4xx formats
			case video_format::VIDEO_FORMAT_YUVA:
				fill_plane(frame, 3, 255); // Pre-fill alpha plane
			case video_format::VIDEO_FORMAT_I444:
				insert_planar_4xx(src, frame, false, false);
				break;
			case video_format::VIDEO_FORMAT_I42A:
				fill_plane(frame, 3, 255); // Pre-fill alpha plane
			case video_format::VIDEO_FORMAT_I422:
				insert_planar_4xx(src, frame, true, false);
				break;
			case video_format::VIDEO_FORMAT_I40A:
				fill_plane(frame, 3, 255); // Pre-fill alpha plane
			case video_format::VIDEO_FORMAT_I420:
				insert_planar_4xx(src, frame, true, true);
				break;

			// Semi-planar NV12 format
			case video_format::VIDEO_FORMAT_NV12:
				insert_semi_planar_nv12(src, frame);
				break;

			// Packed 42x formats
			case video_format::VIDEO_FORMAT_YVYU:
				insert_packed_422(src, frame, true, false);
				break;
			case video_format::VIDEO_FORMAT_YUY2:
				insert_packed_422(src, frame, true, true);
				break;
			case video_format::VIDEO_FORMAT_UYVY:
				insert_packed_422(src, frame, false, true);
				break;

			// Packed uncompressed non-YUV formats
			case video_format::VIDEO_FORMAT_Y800:
				insert_packed_direct_stepped(src, frame, cv::COLOR_YUV2BGR, cv::COLOR_BGR2GRAY);
				break;
			case video_format::VIDEO_FORMAT_RGBA:
				insert_packed_direct_stepped(src, frame, cv::COLOR_YUV2RGB, cv::COLOR_RGB2RGBA);
				break;
			case video_format::VIDEO_FORMAT_BGRX:
			case video_format::VIDEO_FORMAT_BGRA:
				insert_packed_direct_stepped(src, frame, cv::COLOR_YUV2BGR, cv::COLOR_BGR2BGRA);
				break;
			case video_format::VIDEO_FORMAT_BGR3:
				insert_packed_direct(src, frame, cv::COLOR_YUV2BGR);
				break;

			// Unsupported formats
			default:
				return;
		}

	}

	//-------------------------------------------------------------------------------------
}

