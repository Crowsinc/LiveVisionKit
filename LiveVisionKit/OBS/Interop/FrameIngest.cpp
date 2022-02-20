//    *************************** LiveVisionKit ****************************
//    Copyright (C) 2022  Sebastian Di Marco (crowsinc.dev@gmail.com)
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.
// 	  **********************************************************************

#include "FrameIngest.hpp"

#include "Diagnostics/Assert.hpp"

/* NOTE: All upload conversion operations are to YUV, and are performed on the GPU using thread
 * local cached buffers to maximise performance and avoid expensive GPU memory allocations.
 * Thread local is used just in case the API is ever accessed via multiple OBS threads at some point.
 *
 * We assume that the format being upload/downloaded is unlikely to change during a stream
 * so only a subset of the cached buffers will be utilised, without a need to be resized
 * or re-allocated between downloads/uploads. There are a lot of buffers involved but the
 * GPU memory is less important than minimising the overhead of the OBS frame to OpenCV mat
 * conversion, which is a slow and unproductive but necessary operation.
 *
 * We prefer YUV over BGR because many vision algorithms require only the Y plane.
 * Additionally, OBS uses a color matrix to convert YUV to RGB which makes OpenCV's
 * YUV to RGB conversion result in different colors than OBS.
 */

//---------------------------------------------------------------------------------------------------------------------

bool operator<<(cv::UMat& dst, const obs_source_frame* src)
{
	return lvk::import_frame(src, dst);
}

//---------------------------------------------------------------------------------------------------------------------

bool operator>>(const cv::UMat& src, obs_source_frame* dst)
{
	return lvk::export_frame(src, dst);
}

//---------------------------------------------------------------------------------------------------------------------

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	bool is_frame_initialised(const obs_source_frame& frame)
	{
		return frame.data[0] != nullptr
			&& frame.width > 0
			&& frame.height > 0
			&& frame.linesize[0] >= frame.width
			&& frame.format != VIDEO_FORMAT_NONE;
	}

//---------------------------------------------------------------------------------------------------------------------

	void fill_plane(obs_source_frame& dst, const uint32_t plane, const uint8_t value)
	{
		LVK_ASSERT(plane < MAX_AV_PLANES);
		LVK_ASSERT(is_frame_initialised(dst));
		LVK_ASSERT(dst.data[plane] != nullptr);

		std::memset(dst.data[plane], value, dst.width * dst.height);
	}

//---------------------------------------------------------------------------------------------------------------------

	void merge_planes(const cv::UMat& p1, const cv::UMat& p2, const cv::UMat& p3, cv::UMat& dst)
	{
		LVK_ASSERT(p1.type() == CV_8UC1 && p2.type() == CV_8UC1 && p3.type() == CV_8UC1);
		LVK_ASSERT(!p1.empty() && !p2.empty() && !p3.empty());

		cv::merge(std::vector<cv::UMat>{p1, p2, p3}, dst);
	}

//---------------------------------------------------------------------------------------------------------------------

	void merge_planes(const cv::UMat& p1, const cv::UMat& p2, cv::UMat& dst)
	{
		LVK_ASSERT(p1.type() == CV_8UC1 && p2.type() == CV_8UC1);
		LVK_ASSERT(!p1.empty() && !p2.empty());

		cv::merge(std::vector<cv::UMat>{p1, p2}, dst);
	}

//---------------------------------------------------------------------------------------------------------------------

	void split_planes(const cv::UMat& src, cv::UMat& p1, cv::UMat& p2, cv::UMat& p3)
	{
		LVK_ASSERT(!src.empty() && src.type() == CV_8UC3);

		p1.create(src.size(), CV_8UC1, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		p2.create(src.size(), CV_8UC1, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		p3.create(src.size(), CV_8UC1, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		cv::split(src, std::vector<cv::UMat>{p1, p2, p3});
	}

//---------------------------------------------------------------------------------------------------------------------

	void split_planes(const cv::UMat& src, cv::UMat& p1, cv::UMat& p2)
	{
		LVK_ASSERT(!src.empty() && src.type() == CV_8UC2);

		p1.create(src.size(), CV_8UC1, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		p2.create(src.size(), CV_8UC1, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		cv::split(src, std::vector<cv::UMat>{p1, p2});
	}

//---------------------------------------------------------------------------------------------------------------------

	void import_data(
		uint8_t* src,
		cv::UMat& dst,
		const uint32_t width,
		const uint32_t height,
		const uint32_t line_size,
		const uint32_t components
	)
	{
		LVK_ASSERT(src != nullptr);
		LVK_ASSERT(width > 0 && height > 0);
		LVK_ASSERT(components > 0 && components <= 4);
		LVK_ASSERT(line_size >= width * components);

		// NOTE: This is one of the slowest parts of all vision filters, so speeding this up would speed up everything.
		cv::Mat(height, width, CV_8UC(components), src, line_size).copyTo(dst);
	}

//---------------------------------------------------------------------------------------------------------------------

	void export_data(const cv::UMat& src, uint8_t* dst)
	{
		LVK_ASSERT(!src.empty() && dst != nullptr);

		// NOTE: This assumes that the destination is large enough to actually hold all the src data.

		// NOTE: This is one of the slowest parts of all vision filters, so speeding this up would speed up everything.
		src.copyTo(cv::Mat(src.size(), src.type(), dst));
	}

//---------------------------------------------------------------------------------------------------------------------

	void import_plane(
		const obs_source_frame& src,
		cv::UMat& dst,
		const uint32_t plane,
		const float width_scaling,
		const float height_scaling,
		const uint32_t components
	)
	{
		LVK_ASSERT(plane < MAX_AV_PLANES);
		LVK_ASSERT(is_frame_initialised(src));
		LVK_ASSERT(width_scaling * src.width > 0.0 && width_scaling * src.width <= src.width);
		LVK_ASSERT(height_scaling * src.height > 0.0 && height_scaling * src.height <= src.height);

		import_data(
			src.data[plane],
			dst,
			src.width * width_scaling,
			src.height * height_scaling,
			src.linesize[plane],
			components
		);
	}

//---------------------------------------------------------------------------------------------------------------------

	void export_plane(const cv::UMat& src, obs_source_frame& dst, const uint32_t plane)
	{
		LVK_ASSERT(!src.empty());
		LVK_ASSERT(plane < MAX_AV_PLANES);
		LVK_ASSERT(is_frame_initialised(dst));

		export_data(src, dst.data[plane]);
		dst.linesize[plane] = src.step;
	}

//---------------------------------------------------------------------------------------------------------------------

	void import_planar_4xx(
		const obs_source_frame& src,
		cv::UMat& dst,
		const bool subsampled_width,
		const bool subsampled_height
	)
	{
		LVK_ASSERT(is_frame_initialised(src));
		LVK_ASSERT(src.data[0] != nullptr && src.data[1] != nullptr && src.data[2] != nullptr);

		thread_local cv::UMat buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_y(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_u(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_v(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		import_plane(src, plane_y, 0, 1.0f, 1.0f, 1);

		if(subsampled_width || subsampled_height)
		{
			const float chroma_width_scale = subsampled_width ? 0.5 : 1.0;
			const float chroma_height_scale = subsampled_height ? 0.5 : 1.0;

			import_plane(src, buffer, 1, chroma_width_scale, chroma_height_scale, 1);
			cv::resize(buffer, plane_u, plane_y.size(), 0, 0, cv::INTER_LINEAR);

			import_plane(src, buffer, 2, chroma_width_scale, chroma_height_scale, 1);
			cv::resize(buffer, plane_v, plane_y.size(), 0, 0, cv::INTER_LINEAR);
		}
		else
		{
			import_plane(src, plane_u, 1, 1.0, 1.0, 1);
			import_plane(src, plane_v, 2, 1.0, 1.0, 1);
		}

		merge_planes(plane_y, plane_u, plane_v, dst);
	}

//---------------------------------------------------------------------------------------------------------------------

	void export_planar_4xx(
		const cv::UMat& src,
		obs_source_frame& dst,
		const bool subsample_width,
		const bool subsample_height
	)
	{
		LVK_ASSERT(is_frame_initialised(dst));
		LVK_ASSERT(dst.data[0] != nullptr && dst.data[1] != nullptr && dst.data[2] != nullptr);
		LVK_ASSERT(dst.width >= (uint32_t)src.cols && dst.height >= (uint32_t)src.rows);
		LVK_ASSERT(!src.empty() && src.type() == CV_8UC3);

		thread_local cv::UMat buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_y(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_u(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_v(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		split_planes(src, plane_y, plane_u, plane_v);

		export_plane(plane_y, dst, 0);

		if(subsample_width || subsample_height)
		{
			const auto chroma_width_scale = subsample_width ? 0.5 : 1.0;
			const auto chroma_height_scale = subsample_height ? 0.5 : 1.0;

			cv::resize(plane_u, buffer, cv::Size(), chroma_width_scale, chroma_height_scale, cv::INTER_AREA);
			export_plane(buffer, dst, 1);

			cv::resize(plane_v, buffer, cv::Size(), chroma_width_scale, chroma_height_scale, cv::INTER_AREA);
			export_plane(buffer, dst, 2);
		}
		else
		{
			export_plane(plane_u, dst, 1);
			export_plane(plane_v, dst, 2);
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void import_semi_planar_nv12(const obs_source_frame& src, cv::UMat& dst)
	{
		LVK_ASSERT(is_frame_initialised(src));
		LVK_ASSERT(src.data[0] != nullptr && src.data[1] != nullptr);

		thread_local cv::UMat buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_y(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_uv(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		// NOTE: OpenCV provides 'cvtColorTwoPlane' for directly converting NVXX,
		// but the function takes around ~4ms to run (January 2022).

		import_plane(src, plane_y, 0, 1.0, 1.0, 1);

		import_plane(src, buffer, 1, 0.5, 0.5, 2);
		cv::resize(buffer, plane_uv, plane_y.size(), 0, 0, cv::INTER_LINEAR);

		dst.create(plane_y.size(), CV_8UC3, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		cv::mixChannels({{plane_y, plane_uv}}, std::vector<cv::UMat>{dst}, {0,0,  1,1,  2,2});
	}

//---------------------------------------------------------------------------------------------------------------------

	void export_semi_planar_nv12(const cv::UMat& src, obs_source_frame& dst)
	{
		LVK_ASSERT(is_frame_initialised(dst));
		LVK_ASSERT(dst.data[0] != nullptr && dst.data[1] != nullptr);
		LVK_ASSERT(dst.width >= (uint32_t)src.cols && dst.height >= (uint32_t)src.rows);
		LVK_ASSERT(!src.empty() && src.type() == CV_8UC3);

		thread_local cv::UMat buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_y(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_uv(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		cv::extractChannel(src, plane_y, 0);
		export_plane(plane_y, dst, 0);

		buffer.create(src.size(), CV_8UC2, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		cv::mixChannels({src}, std::vector<cv::UMat>{buffer}, {1,0,  2,1});

		cv::resize(buffer, plane_uv, cv::Size(), 0.5, 0.5, cv::INTER_AREA);

		export_plane(plane_uv, dst, 1);
	}

//---------------------------------------------------------------------------------------------------------------------

	void import_packed_422(const obs_source_frame& src, cv::UMat& dst, const bool y_first, const bool u_first)
	{
		LVK_ASSERT(is_frame_initialised(src));
		LVK_ASSERT(src.data[0] != nullptr);

		thread_local cv::UMat buffer_1(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat buffer_2(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_uv(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		import_plane(src, buffer_1, 0, 1.0, 1.0, 2);
		cv::extractChannel(buffer_1, buffer_2, y_first ? 1 : 0);

		buffer_2 = buffer_2.reshape(2, buffer_2.rows);
		cv::resize(buffer_2, plane_uv, buffer_1.size(), 0, 0, cv::INTER_LINEAR);

		std::vector<int> from_to(6);
		if(u_first)
			from_to = {(y_first ? 0 : 1),0,  2,1,  3,2};
		else
			from_to = {(y_first ? 0 : 1),0,  2,2,  3,1};

		dst.create(buffer_1.size(), CV_8UC3, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		cv::mixChannels({{buffer_1, plane_uv}}, std::vector<cv::UMat>{dst}, from_to);
	}

//---------------------------------------------------------------------------------------------------------------------

	void export_packed_422(const cv::UMat& src, obs_source_frame& dst, const bool y_first, const bool u_first)
	{
		LVK_ASSERT(is_frame_initialised(dst));
		LVK_ASSERT(dst.width >= (uint32_t)src.cols && dst.height >= (uint32_t)src.rows);
		LVK_ASSERT(!src.empty() && src.type() == CV_8UC3);
		LVK_ASSERT(dst.data[0] != nullptr);

		thread_local cv::UMat buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_y(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_uv(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		cv::extractChannel(src, plane_y, 0);

		buffer.create(src.size(), CV_8UC2, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		if(u_first)
			cv::mixChannels({src}, std::vector<cv::UMat>{buffer}, {1,0,  2,1});
		else
			cv::mixChannels({src}, std::vector<cv::UMat>{buffer}, {2,0,  1,1});

		cv::resize(buffer, plane_uv, cv::Size(), 0.5, 1.0, cv::INTER_AREA);
		plane_uv = plane_uv.reshape(1, plane_uv.rows);

		if(y_first)
			cv::mixChannels({{plane_y, plane_uv}}, std::vector<cv::UMat>{buffer}, {0,0,  1,1});
		else
			cv::mixChannels({{plane_y, plane_uv}}, std::vector<cv::UMat>{buffer}, {0,1,  1,0});

		export_plane(buffer, dst, 0);
	}

//---------------------------------------------------------------------------------------------------------------------

	void import_packed_444(const obs_source_frame& src, cv::UMat& dst, const bool has_alpha)
	{
		LVK_ASSERT(is_frame_initialised(src));
		LVK_ASSERT(src.data[0] != nullptr);

		thread_local cv::UMat buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		if(has_alpha)
		{
			import_data(
				src.data[0],
				buffer,
				src.width,
				src.height,
				src.linesize[0],
				4
			);

			dst.create(buffer.size(), CV_8UC3, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
			cv::mixChannels({buffer}, std::vector<cv::UMat>{dst}, {1,0,  2,1,  3,2});
		}
		else
		{
			import_data(
				src.data[0],
				dst,
				src.width,
				src.height,
				src.linesize[0],
				3
			);
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void export_packed_444(const cv::UMat& src, obs_source_frame& dst, const bool has_alpha)
	{
		LVK_ASSERT(is_frame_initialised(dst));
		LVK_ASSERT(dst.width >= (uint32_t)src.cols && dst.height >= (uint32_t)src.rows);
		LVK_ASSERT(dst.data[0] != nullptr);
		LVK_ASSERT(!src.empty());

		thread_local cv::UMat buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat alpha_buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		if(has_alpha)
		{
			// NOTE: To preserve alpha we need to import the frame and mix the
			// original alpha channel back in. This is slow, don't use AYUV.

			import_data(
				dst.data[0],
				alpha_buffer,
				dst.width,
				dst.height,
				dst.linesize[0],
				4
			);

			buffer.create(src.size(), CV_8UC4, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
			cv::mixChannels({{src, alpha_buffer}}, std::vector<cv::UMat>{buffer}, {3,0,  0,1,  1,2,  2,3});

			export_plane(buffer, dst, 0);
		} else export_plane(src, dst, 0);
	}

//---------------------------------------------------------------------------------------------------------------------

	void import_packed_direct_stepped(
		const obs_source_frame& src,
		cv::UMat& dst,
		const uint32_t components,
		const cv::ColorConversionCodes conversion_1,
		const cv::ColorConversionCodes conversion_2
	)
	{
		LVK_ASSERT(components > 0 && components <= 4);
		LVK_ASSERT(is_frame_initialised(src));
		LVK_ASSERT(src.data[0] != nullptr);

		thread_local cv::UMat buffer_1(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat buffer_2(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		import_data(
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

//---------------------------------------------------------------------------------------------------------------------

	void export_packed_direct_stepped(
		const cv::UMat& src,
		obs_source_frame& dst,
		const cv::ColorConversionCodes conversion_1,
		const cv::ColorConversionCodes conversion_2
	)
	{
		LVK_ASSERT(is_frame_initialised(dst));
		LVK_ASSERT(dst.width >= (uint32_t)src.cols && dst.height >= (uint32_t)src.rows);
		LVK_ASSERT(dst.data[0] != nullptr);
		LVK_ASSERT(!src.empty());

		thread_local cv::UMat buffer_1(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat buffer_2(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		cv::cvtColor(src, buffer_1, conversion_1);
		cv::cvtColor(buffer_1, buffer_2, conversion_2);
		export_plane(buffer_2, dst, 0);
	}

//---------------------------------------------------------------------------------------------------------------------

	void import_packed_direct(
		const obs_source_frame& src,
		cv::UMat& dst,
		const uint32_t components,
		const cv::ColorConversionCodes conversion
	){
		LVK_ASSERT(components > 0 && components <= 4);
		LVK_ASSERT(is_frame_initialised(src));
		LVK_ASSERT(src.data[0] != nullptr);

		thread_local cv::UMat buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		import_data(
			src.data[0],
			buffer,
			src.width,
			src.height,
			src.linesize[0],
			components
		);

		cv::cvtColor(buffer, dst, conversion);
	}

//---------------------------------------------------------------------------------------------------------------------

	void export_packed_direct(const cv::UMat& src, obs_source_frame& dst, const cv::ColorConversionCodes conversion)
	{
		LVK_ASSERT(is_frame_initialised(dst));
		LVK_ASSERT(dst.width >= (uint32_t)src.cols && dst.height >= (uint32_t)src.rows);
		LVK_ASSERT(dst.data[0] != nullptr);
		LVK_ASSERT(!src.empty());

		thread_local cv::UMat buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		cv::cvtColor(src, buffer, conversion);
		export_plane(buffer, dst, 0);
	}

//---------------------------------------------------------------------------------------------------------------------

	bool import_frame(const obs_source_frame* src, cv::UMat& dst)
	{
		LVK_ASSERT(src != nullptr);
		LVK_ASSERT(is_frame_initialised(*src));

		const auto& frame = *src;

		switch(frame.format)
		{
			// Planar 4xx formats
			case video_format::VIDEO_FORMAT_YUVA:
			case video_format::VIDEO_FORMAT_I444:
				import_planar_4xx(frame, dst, false, false);
				break;
			case video_format::VIDEO_FORMAT_I42A:
			case video_format::VIDEO_FORMAT_I422:
				import_planar_4xx(frame, dst, true, false);
				break;
			case video_format::VIDEO_FORMAT_I40A:
			case video_format::VIDEO_FORMAT_I420:
				import_planar_4xx(frame, dst, true, true);
				break;

			// Semi-planar NV12 format
			case video_format::VIDEO_FORMAT_NV12:
				import_semi_planar_nv12(frame, dst);
				break;

			// Packed 42x YUV formats
			case video_format::VIDEO_FORMAT_YVYU:
				import_packed_422(frame, dst, true, false);
				break;
			case video_format::VIDEO_FORMAT_YUY2:
				import_packed_422(frame, dst, true, true);
				break;
			case video_format::VIDEO_FORMAT_UYVY:
				import_packed_422(frame, dst, false, true);
				break;

			// Packed 444 YUV formats
			case video_format::VIDEO_FORMAT_AYUV:
				import_packed_444(frame, dst, true);
				break;

			// Packed uncompressed non-YUV formats
			case video_format::VIDEO_FORMAT_Y800:
				import_packed_direct_stepped(frame, dst, 1, cv::COLOR_GRAY2BGR, cv::COLOR_BGR2YUV);
				break;
			case video_format::VIDEO_FORMAT_RGBA:
				import_packed_direct_stepped(frame, dst, 4, cv::COLOR_RGBA2RGB, cv::COLOR_RGB2YUV);
				break;
			case video_format::VIDEO_FORMAT_BGRX:
			case video_format::VIDEO_FORMAT_BGRA:
				import_packed_direct_stepped(frame, dst, 4, cv::COLOR_BGRA2BGR, cv::COLOR_BGR2YUV);
				break;
			case video_format::VIDEO_FORMAT_BGR3:
				import_packed_direct(frame, dst, 3, cv::COLOR_BGR2YUV);
				break;

			// Unsupported formats
			default:
				LVK_ASSERT(false && "Unsupported Format (please ask LVK developers to add support)");
				return false;
		}
		return true;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool export_frame(const cv::UMat& src, obs_source_frame* dst)
	{
		LVK_ASSERT(dst != nullptr);
		LVK_ASSERT(is_frame_initialised(*dst));
		LVK_ASSERT(dst->width >= (uint32_t)src.cols && dst->height >= (uint32_t)src.rows);
		LVK_ASSERT(!src.empty() && src.type() == CV_8UC3);

		auto& frame = *dst;

		switch(frame.format)
		{
			// Planar 4xx formats
			case video_format::VIDEO_FORMAT_YUVA:
			case video_format::VIDEO_FORMAT_I444:
				export_planar_4xx(src, frame, false, false);
				break;
			case video_format::VIDEO_FORMAT_I42A:
			case video_format::VIDEO_FORMAT_I422:
				export_planar_4xx(src, frame, true, false);
				break;
			case video_format::VIDEO_FORMAT_I40A:
			case video_format::VIDEO_FORMAT_I420:
				export_planar_4xx(src, frame, true, true);
				break;

			// Semi-planar NV12 format
			case video_format::VIDEO_FORMAT_NV12:
				export_semi_planar_nv12(src, frame);
				break;

			// Packed 42x formats
			case video_format::VIDEO_FORMAT_YVYU:
				export_packed_422(src, frame, true, false);
				break;
			case video_format::VIDEO_FORMAT_YUY2:
				export_packed_422(src, frame, true, true);
				break;
			case video_format::VIDEO_FORMAT_UYVY:
				export_packed_422(src, frame, false, true);
				break;

			// Packed 444 YUV formats
			case video_format::VIDEO_FORMAT_AYUV:
				export_packed_444(src, frame, true);
				break;

			// Packed uncompressed non-YUV formats
			case video_format::VIDEO_FORMAT_Y800:
				export_packed_direct_stepped(src, frame, cv::COLOR_YUV2BGR, cv::COLOR_BGR2GRAY);
				break;
			case video_format::VIDEO_FORMAT_RGBA:
				export_packed_direct_stepped(src, frame, cv::COLOR_YUV2RGB, cv::COLOR_RGB2RGBA);
				break;
			case video_format::VIDEO_FORMAT_BGRX:
			case video_format::VIDEO_FORMAT_BGRA:
				export_packed_direct_stepped(src, frame, cv::COLOR_YUV2BGR, cv::COLOR_BGR2BGRA);
				break;
			case video_format::VIDEO_FORMAT_BGR3:
				export_packed_direct(src, frame, cv::COLOR_YUV2BGR);
				break;

			// Unsupported formats
			default:
				LVK_ASSERT(false && "Unsupported Format (please ask LVK developers to add support)");
				return false;
		}

		frame.height = src.rows;
		frame.width = src.cols;

		return true;
	}

//---------------------------------------------------------------------------------------------------------------------

}

