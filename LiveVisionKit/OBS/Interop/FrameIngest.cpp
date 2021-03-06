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

#include <tuple>
#include <thread>

#include "Math/Math.hpp"
#include "Diagnostics/Directives.hpp"
#include "OBS/Utility/Logging.hpp"

#ifdef _WIN32
#include <dxgi.h>
#include <d3d11.h>
#include <opencv2/core/directx.hpp>
#else
#include <opencv2/core/opengl.hpp>
#endif

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
		LVK_ASSERT(p1.type() == CV_8UC1);
		LVK_ASSERT(p2.type() == CV_8UC1);
		LVK_ASSERT(p3.type() == CV_8UC1);
		LVK_ASSERT(!p1.empty());
		LVK_ASSERT(!p2.empty());
		LVK_ASSERT(!p3.empty());

		cv::merge(std::vector<cv::UMat>{p1, p2, p3}, dst);
	}

//---------------------------------------------------------------------------------------------------------------------

	void merge_planes(const cv::UMat& p1, const cv::UMat& p2, cv::UMat& dst)
	{
		LVK_ASSERT(p1.type() == CV_8UC1);
		LVK_ASSERT(p2.type() == CV_8UC1);
		LVK_ASSERT(!p1.empty());
		LVK_ASSERT(!p2.empty());

		cv::merge(std::vector<cv::UMat>{p1, p2}, dst);
	}

//---------------------------------------------------------------------------------------------------------------------

	void split_planes(const cv::UMat& src, cv::UMat& p1, cv::UMat& p2, cv::UMat& p3)
	{
		LVK_ASSERT(src.type() == CV_8UC3);
		LVK_ASSERT(!src.empty());

		p1.create(src.size(), CV_8UC1, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		p2.create(src.size(), CV_8UC1, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		p3.create(src.size(), CV_8UC1, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		cv::split(src, std::vector<cv::UMat>{p1, p2, p3});
	}

//---------------------------------------------------------------------------------------------------------------------

	void split_planes(const cv::UMat& src, cv::UMat& p1, cv::UMat& p2)
	{
		LVK_ASSERT(!src.empty());
		LVK_ASSERT(src.type() == CV_8UC2);

		p1.create(src.size(), CV_8UC1, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		p2.create(src.size(), CV_8UC1, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		cv::split(src, std::vector<cv::UMat>{p1, p2});
	}

//---------------------------------------------------------------------------------------------------------------------

	//NOTE: Returns ROI to internally uploaded planes, planes should be cloned for modification
	cv::UMat upload_planes(
		const obs_source_frame& src,
		const cv::Size plane_0_size,
		const uint32_t plane_0_channels
	)
	{
		LVK_ASSERT(is_frame_initialised(src));
		LVK_ASSERT(between<uint32_t>(plane_0_channels, 1, 4));
		LVK_ASSERT(between<uint32_t>(plane_0_size.width, 1, src.width));
		LVK_ASSERT(between<uint32_t>(plane_0_size.height, 1, src.height));

		thread_local cv::UMat import_buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		const uint64_t import_length = plane_0_size.area() * plane_0_channels;

		cv::Mat(1, import_length, CV_8UC1, src.data[0]).copyTo(import_buffer);

		return import_buffer.reshape(plane_0_channels, plane_0_size.height);
	}

//---------------------------------------------------------------------------------------------------------------------

	//NOTE: Returns ROI to internally uploaded planes, planes should be cloned for modification
	 std::tuple<cv::UMat, cv::UMat>&  upload_planes(
		const obs_source_frame& src,
		const cv::Size plane_0_size,
		const uint32_t plane_0_channels,
		const cv::Size plane_1_size,
		const uint32_t plane_1_channels
	)
	{
		LVK_ASSERT(src.data[0] != nullptr);
		LVK_ASSERT(src.data[1] != nullptr);
		LVK_ASSERT(is_frame_initialised(src));
		LVK_ASSERT(between<uint32_t>(plane_0_channels, 1, 4));
		LVK_ASSERT(between<uint32_t>(plane_0_size.width, 1, src.width));
		LVK_ASSERT(between<uint32_t>(plane_0_size.height, 1, src.height));
		LVK_ASSERT(between<uint32_t>(plane_1_channels, 1, 4));
		LVK_ASSERT(between<uint32_t>(plane_1_size.width, 1, src.width));
		LVK_ASSERT(between<uint32_t>(plane_1_size.height, 1, src.height));


		// NOTE: Uploads are done in bulk by utilising the fact that the OBS planes
		// are all stored in one contiguous span of memory starting at src.data[0].
		// Padding exists between planes which must be avoided.

		thread_local cv::UMat import_buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local std::tuple<cv::UMat, cv::UMat> plane_regions;

		const uint64_t plane_0_length = plane_0_size.area() * plane_0_channels;
		const uint64_t plane_1_length = plane_1_size.area() * plane_1_channels;

		const uint64_t plane_1_offset =  src.data[1] - src.data[0];
		const uint64_t import_length = plane_1_offset + plane_1_length;

		cv::Mat(1, import_length, CV_8UC1, src.data[0]).copyTo(import_buffer);

		std::get<0>(plane_regions) = import_buffer.colRange(0, plane_0_length)
				.reshape(plane_0_channels, plane_0_size.height);

		std::get<1>(plane_regions) = import_buffer.colRange(plane_1_offset, plane_1_offset + plane_1_length)
				.reshape(plane_1_channels, plane_1_size.height);

		return plane_regions;
	}

//---------------------------------------------------------------------------------------------------------------------

	//NOTE: Returns ROI to internally uploaded planes, planes should be cloned for modification
	std::tuple<cv::UMat, cv::UMat, cv::UMat>& upload_planes(
		const obs_source_frame& src,
		const cv::Size plane_0_size,
		const uint32_t plane_0_channels,
		const cv::Size plane_1_size,
		const uint32_t plane_1_channels,
		const cv::Size plane_2_size,
		const uint32_t plane_2_channels
	){
		LVK_ASSERT(src.data[0] != nullptr);
		LVK_ASSERT(src.data[1] != nullptr);
		LVK_ASSERT(src.data[2] != nullptr);
		LVK_ASSERT(is_frame_initialised(src));
		LVK_ASSERT(between<uint32_t>(plane_0_channels, 1, 4));
		LVK_ASSERT(between<uint32_t>(plane_0_size.width, 1, src.width));
		LVK_ASSERT(between<uint32_t>(plane_0_size.height, 1, src.height));
		LVK_ASSERT(between<uint32_t>(plane_1_channels, 1, 4));
		LVK_ASSERT(between<uint32_t>(plane_1_size.width, 1, src.width));
		LVK_ASSERT(between<uint32_t>(plane_1_size.height, 1, src.height));
		LVK_ASSERT(between<uint32_t>(plane_2_channels, 1, 4));
		LVK_ASSERT(between<uint32_t>(plane_2_size.width, 1, src.width));
		LVK_ASSERT(between<uint32_t>(plane_2_size.height, 1, src.height));

		// NOTE: Uploads are done in bulk by utilising the fact that the OBS planes
		// are all stored in one contiguous span of memory starting at src.data[0].
		// Padding exists between planes which must be avoided.

		thread_local cv::UMat import_buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local std::tuple<cv::UMat, cv::UMat, cv::UMat> plane_regions;

		const uint64_t plane_0_length = plane_0_size.area() * plane_0_channels;
		const uint64_t plane_1_length = plane_1_size.area() * plane_1_channels;
		const uint64_t plane_2_length = plane_2_size.area() * plane_2_channels;

		const uint64_t plane_1_offset =  src.data[1] - src.data[0];
		const uint64_t plane_2_offset =  src.data[2] - src.data[0];
		const uint64_t import_length = plane_2_offset + plane_2_length;

		cv::Mat(1, import_length, CV_8UC1, src.data[0]).copyTo(import_buffer);

		std::get<0>(plane_regions) = import_buffer.colRange(0, plane_0_length)
				.reshape(plane_0_channels, plane_0_size.height);

		std::get<1>(plane_regions) = import_buffer.colRange(plane_1_offset, plane_1_offset + plane_1_length)
				.reshape(plane_1_channels, plane_1_size.height);

		std::get<2>(plane_regions) = import_buffer.colRange(plane_2_offset, plane_2_offset + plane_2_length)
				.reshape(plane_2_channels, plane_2_size.height);

		return plane_regions;
	}

//---------------------------------------------------------------------------------------------------------------------

	//NOTE: Returns ROI to internally uploaded planes, planes should be cloned for modification
	cv::UMat upload_planes(const obs_source_frame& src, const uint32_t channels)
	{
		return upload_planes(src, cv::Size(src.width, src.height), channels);
	}

//---------------------------------------------------------------------------------------------------------------------

	void download_planes(
		const cv::UMat plane_0,
		obs_source_frame& dst
	){
		LVK_ASSERT(!plane_0.empty());
		LVK_ASSERT(is_frame_initialised(dst));
		LVK_ASSERT(between<uint32_t>(plane_0.cols, 1, dst.width));
		LVK_ASSERT(between<uint32_t>(plane_0.rows, 1, dst.height));

		const uint64_t export_length = plane_0.total() * plane_0.elemSize();

		plane_0.reshape(1, 1).copyTo(cv::Mat(1, export_length, CV_8UC1, dst.data[0]));
	}

//---------------------------------------------------------------------------------------------------------------------

	void download_planes(
		const cv::UMat plane_0,
		const cv::UMat plane_1,
		obs_source_frame& dst
	){
		LVK_ASSERT(!plane_0.empty())
		LVK_ASSERT(!plane_1.empty());
		LVK_ASSERT(dst.data[0] != nullptr);
		LVK_ASSERT(dst.data[1] != nullptr);
		LVK_ASSERT(is_frame_initialised(dst));
		LVK_ASSERT(between<uint32_t>(plane_0.cols, 1, dst.width));
		LVK_ASSERT(between<uint32_t>(plane_0.rows, 1, dst.height));
		LVK_ASSERT(between<uint32_t>(plane_1.cols, 1, dst.width));
		LVK_ASSERT(between<uint32_t>(plane_1.rows, 1, dst.height));

		// NOTE: Downloads are done in bulk by utilising the fact that the OBS planes
		// are all stored in one contiguous span of memory starting at src.data[0].
		// We must conserve the padding which exists between planes in memory.

		thread_local cv::UMat export_buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		const uint64_t plane_0_length = plane_0.total() * plane_0.elemSize();
		const uint64_t plane_1_length = plane_1.total() * plane_1.elemSize();

		const uint64_t plane_1_offset =  dst.data[1] - dst.data[0];
		const uint64_t export_length = plane_1_offset + plane_1_length;

		export_buffer.create(1, export_length, CV_8UC1);

		plane_0.reshape(1, 1).copyTo(export_buffer.colRange(0, plane_0_length));
		plane_1.reshape(1, 1).copyTo(export_buffer.colRange(plane_1_offset, plane_1_offset + plane_1_length));

		export_buffer.copyTo(cv::Mat(1, export_length, CV_8UC1, dst.data[0]));
	}

//---------------------------------------------------------------------------------------------------------------------

	void download_planes(
		const cv::UMat plane_0,
		const cv::UMat plane_1,
		const cv::UMat plane_2,
		obs_source_frame& dst
	){
		LVK_ASSERT(!plane_0.empty());
		LVK_ASSERT(!plane_1.empty());
		LVK_ASSERT(!plane_2.empty());
		LVK_ASSERT(dst.data[0] != nullptr);
		LVK_ASSERT(dst.data[1] != nullptr);
		LVK_ASSERT(dst.data[2] != nullptr);
		LVK_ASSERT(is_frame_initialised(dst));
		LVK_ASSERT(between<uint32_t>(plane_0.cols, 1, dst.width));
		LVK_ASSERT(between<uint32_t>(plane_0.rows, 1, dst.height));
		LVK_ASSERT(between<uint32_t>(plane_1.cols, 1, dst.width));
		LVK_ASSERT(between<uint32_t>(plane_1.rows, 1, dst.height));
		LVK_ASSERT(between<uint32_t>(plane_2.cols, 1, dst.width));
		LVK_ASSERT(between<uint32_t>(plane_2.rows, 1, dst.height));

		// NOTE: Downloads are done in bulk by utilising the fact that the OBS planes
		// are all stored in one contiguous span of memory starting at src.data[0].
		// We must conserve the padding which exists between planes in memory.

		thread_local cv::UMat export_buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		const uint64_t plane_0_length = plane_0.total() * plane_0.elemSize();
		const uint64_t plane_1_length = plane_1.total() * plane_1.elemSize();
		const uint64_t plane_2_length = plane_2.total() * plane_2.elemSize();

		const uint64_t plane_1_offset =  dst.data[1] - dst.data[0];
		const uint64_t plane_2_offset =  dst.data[2] - dst.data[0];
		const uint64_t export_length = plane_2_offset + plane_2_length;

		export_buffer.create(1, export_length, CV_8UC1);

		plane_0.reshape(1, 1).copyTo(export_buffer.colRange(0, plane_0_length));
		plane_1.reshape(1, 1).copyTo(export_buffer.colRange(plane_1_offset, plane_1_offset + plane_1_length));
		plane_2.reshape(1, 1).copyTo(export_buffer.colRange(plane_2_offset, plane_2_offset + plane_2_length));

		export_buffer.copyTo(cv::Mat(1, export_length, CV_8UC1, dst.data[0]));
	}

//---------------------------------------------------------------------------------------------------------------------

	void import_planar_4xx(
		const obs_source_frame& src,
		cv::UMat& dst,
		const bool subsampled_width,
		const bool subsampled_height
	)
	{
		thread_local cv::UMat plane_u_buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_v_buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		const cv::Size frame_size(src.width, src.height);
		const cv::Size chroma_size(
			(subsampled_width ? 0.5 : 1.0) * frame_size.width,
			(subsampled_height ? 0.5 : 1.0) * frame_size.height
		);

		auto& [y_roi, u_roi, v_roi] = upload_planes(
			src,
			frame_size, 1,
			chroma_size, 1,
			chroma_size, 1
		);

		LVK_ASSERT(!y_roi.empty());
		LVK_ASSERT(!u_roi.empty());
		LVK_ASSERT(!v_roi.empty());

		if(subsampled_width || subsampled_height)
		{
			cv::resize(u_roi, plane_u_buffer, frame_size, 0, 0, cv::INTER_LINEAR);
			cv::resize(v_roi, plane_v_buffer, frame_size, 0, 0, cv::INTER_LINEAR);

			merge_planes(y_roi, plane_u_buffer, plane_v_buffer, dst);
		}
		else merge_planes(y_roi, u_roi, v_roi, dst);
	}

//---------------------------------------------------------------------------------------------------------------------

	void export_planar_4xx(
		const cv::UMat& src,
		obs_source_frame& dst,
		const bool subsample_width,
		const bool subsample_height
	)
	{
		thread_local cv::UMat plane_y(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_u(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_v(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		split_planes(src, plane_y, plane_u, plane_v);

		if(subsample_width || subsample_height)
		{
			thread_local cv::UMat plane_sub_u(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
			thread_local cv::UMat plane_sub_v(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

			const auto chroma_width_scale = subsample_width ? 0.5 : 1.0;
			const auto chroma_height_scale = subsample_height ? 0.5 : 1.0;

			cv::resize(plane_u, plane_sub_u, cv::Size(), chroma_width_scale, chroma_height_scale, cv::INTER_AREA);
			cv::resize(plane_v, plane_sub_v, cv::Size(), chroma_width_scale, chroma_height_scale, cv::INTER_AREA);

			download_planes(plane_y, plane_sub_u, plane_sub_v, dst);
		}
		else download_planes(plane_y, plane_u, plane_v, dst);
	}

//---------------------------------------------------------------------------------------------------------------------

	void import_semi_planar_nv12(const obs_source_frame& src, cv::UMat& dst)
	{
		thread_local cv::UMat plane_uv(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		const cv::Size frame_size(src.width, src.height);
		const cv::Size chroma_size(src.width * 0.5, src.height * 0.5);

		auto& [y_roi, uv_roi] = upload_planes(
			src,
			frame_size, 1,
			chroma_size, 2
		);

		cv::resize(uv_roi, plane_uv, frame_size, 0, 0, cv::INTER_LINEAR);
		dst.create(frame_size, CV_8UC3, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		cv::mixChannels({{y_roi, plane_uv}}, std::vector<cv::UMat>{dst}, {0,0,  1,1,  2,2});
	}

//---------------------------------------------------------------------------------------------------------------------

	void export_semi_planar_nv12(const cv::UMat& src, obs_source_frame& dst)
	{
		thread_local cv::UMat plane_y(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_uv(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_sub_uv(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		plane_y.create(src.size(), CV_8UC1);
		plane_uv.create(src.size(), CV_8UC2);

		cv::mixChannels({src}, std::vector<cv::UMat>{plane_y, plane_uv}, {0,0, 1,1,  2,2});

		cv::resize(plane_uv, plane_sub_uv, cv::Size(), 0.5, 0.5, cv::INTER_AREA);

		download_planes(plane_y, plane_sub_uv, dst);
	}

//---------------------------------------------------------------------------------------------------------------------

	void import_packed_422(const obs_source_frame& src, cv::UMat& dst, const bool y_first, const bool u_first)
	{
		thread_local cv::UMat plane_sub_uv(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_uv(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		cv::UMat plane_roi = upload_planes(src, 2);

		// Re-interpret uv plane as 2 components to remove interleaving, then upsample to correct size
		cv::extractChannel(plane_roi, plane_sub_uv, y_first ? 1 : 0);
		cv::resize(plane_sub_uv.reshape(2, plane_sub_uv.rows), plane_uv, plane_roi.size(), 0, 0, cv::INTER_LINEAR);

		std::vector<int> from_to(6);
		if(u_first)
			from_to = {(y_first ? 0 : 1),0,  2,1,  3,2};
		else
			from_to = {(y_first ? 0 : 1),0,  2,2,  3,1};

		// Merge upsampled uv plane back with the y plane
		dst.create(plane_roi.size(), CV_8UC3, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		cv::mixChannels({{plane_roi, plane_uv}}, std::vector<cv::UMat>{dst}, from_to);
	}

//---------------------------------------------------------------------------------------------------------------------

	void export_packed_422(const cv::UMat& src, obs_source_frame& dst, const bool y_first, const bool u_first)
	{
		thread_local cv::UMat buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_y(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat plane_uv(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		buffer.create(src.size(), CV_8UC2);

		// Extract uv planes
		if(u_first)
			cv::mixChannels({src}, std::vector<cv::UMat>{buffer}, {1,0,  2,1});
		else
			cv::mixChannels({src}, std::vector<cv::UMat>{buffer}, {2,0,  1,1});

		// Subsample uv plane width and re-interpret as one channel to interleave u and v components
		cv::resize(buffer, plane_uv, cv::Size(), 0.5, 1.0, cv::INTER_AREA);
		plane_uv = plane_uv.reshape(1, plane_uv.rows);

		// Pack y and interleaved uv planes
		cv::extractChannel(src, plane_y, 0);
		if(y_first)
			cv::mixChannels({{plane_y, plane_uv}}, std::vector<cv::UMat>{buffer}, {0,0,  1,1});
		else
			cv::mixChannels({{plane_y, plane_uv}}, std::vector<cv::UMat>{buffer}, {0,1,  1,0});

		download_planes(buffer, dst);
	}

//---------------------------------------------------------------------------------------------------------------------

	void import_packed_444(const obs_source_frame& src, cv::UMat& dst, const bool has_alpha)
	{
		if(has_alpha)
		{
			cv::UMat plane_roi = upload_planes(src, 4);

			dst.create(plane_roi.size(), CV_8UC3, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
			cv::mixChannels({plane_roi}, std::vector<cv::UMat>{dst}, {1,0,  2,1,  3,2});
		}
		else upload_planes(src, 3).copyTo(dst);
	}

//---------------------------------------------------------------------------------------------------------------------

	void export_packed_444(const cv::UMat& src, obs_source_frame& dst, const bool has_alpha)
	{
		if(has_alpha)
		{
			thread_local cv::UMat buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

			// NOTE: To preserve alpha we need to import the frame and mix the
			// original alpha channel back in. This is slow, don't use AYUV.
			cv::UMat dst_roi = upload_planes(dst, 4);

			buffer.create(src.size(), CV_8UC4);
			cv::mixChannels({{src, dst_roi}}, std::vector<cv::UMat>{buffer}, {3,0,  0,1,  1,2,  2,3});

			download_planes(buffer, dst);
		} else download_planes(src, dst);
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
		thread_local cv::UMat converison_buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		cv::cvtColor(upload_planes(src, components), converison_buffer, conversion_1);
		cv::cvtColor(converison_buffer, dst, conversion_2);
	}

//---------------------------------------------------------------------------------------------------------------------

	void export_packed_direct_stepped(
		const cv::UMat& src,
		obs_source_frame& dst,
		const cv::ColorConversionCodes conversion_1,
		const cv::ColorConversionCodes conversion_2
	)
	{
		thread_local cv::UMat buffer_1(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		thread_local cv::UMat buffer_2(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		cv::cvtColor(src, buffer_1, conversion_1);
		cv::cvtColor(buffer_1, buffer_2, conversion_2);
		download_planes(buffer_2, dst);
	}

//---------------------------------------------------------------------------------------------------------------------

	void import_packed_direct(
		const obs_source_frame& src,
		cv::UMat& dst,
		const uint32_t components,
		const cv::ColorConversionCodes conversion
	){
		cv::cvtColor(upload_planes(src, components), dst, conversion);
	}

//---------------------------------------------------------------------------------------------------------------------

	void export_packed_direct(const cv::UMat& src, obs_source_frame& dst, const cv::ColorConversionCodes conversion)
	{
		thread_local cv::UMat buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		cv::cvtColor(src, buffer, conversion);
		download_planes(buffer, dst);
	}

//---------------------------------------------------------------------------------------------------------------------

	void import_frame(const obs_source_frame* src, cv::UMat& dst)
	{
		LVK_ASSERT(src != nullptr);
		LVK_ASSERT(src->format != VIDEO_FORMAT_NONE);

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
				log::error("Frame ingest does not support format %s", get_video_format_name(frame.format));
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void export_frame(const cv::UMat& src, obs_source_frame* dst)
	{
		LVK_ASSERT(dst != nullptr);
		LVK_ASSERT(!src.empty());
		LVK_ASSERT(src.type() == CV_8UC3);
		LVK_ASSERT(dst->format != VIDEO_FORMAT_NONE);

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
				log::error("Frame ingest does not support format %s", get_video_format_name(frame.format));
		}

		frame.height = src.rows;
		frame.width = src.cols;
	}

//---------------------------------------------------------------------------------------------------------------------

}

namespace lvk::ocl
{
//---------------------------------------------------------------------------------------------------------------------

	bool supports_graphics_interop()
	{
		if (!cv::ocl::haveOpenCL())
			return false;

		auto& device = cv::ocl::Device::getDefault();

#ifdef _WIN32
		// DirectX11 
		return device.isExtensionSupported("cl_nv_d3d11_sharing") || device.isExtensionSupported("cl_khr_d3d11_sharing");
#else
		// OpenGL

		// NOTE: Linux OpenGL interop is implemented but disabled due 
		// to suspected driver issues. Needs further testing across
		// multple Linux machines. 
		return false; //device.isExtensionSupported("cl_khr_gl_sharing");
#endif
	}

//---------------------------------------------------------------------------------------------------------------------

	void try_attach_graphics_interop_context()
	{
		static cv::ocl::OpenCLExecutionContext opencl_context;
		static graphics_t* graphics_context = nullptr;
		static std::thread::id bound_thread;
		
		if (opencl_context.empty())
		{
			LVK_ASSERT(gs_get_context() != nullptr);
			LVK_ASSERT(supports_graphics_interop());
#ifdef _WIN32 
			// DirectX11 Context
			auto device = static_cast<ID3D11Device*>(gs_get_device_obj());
			cv::directx::ocl::initializeContextFromD3D11Device(device);
#else       
			// OpenGL Context
			cv::ogl::ocl::initializeContextFromGL();
#endif
			graphics_context = gs_get_context();
			opencl_context = cv::ocl::OpenCLExecutionContext::getCurrent();
			bound_thread = std::this_thread::get_id();
		}

		// NOTE: We are making the assumption that 
		// OBS only ever has one graphics context. 
		LVK_ASSERT(gs_get_context() == graphics_context);
		if (std::this_thread::get_id() != bound_thread)
		{
			log::warn("Switching OpenCL interop context over to new thread");
			bound_thread = std::this_thread::get_id();
			opencl_context.bind();
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void import_texture(gs_texture_t* src, cv::UMat& dst)
	{
		LVK_ASSERT(src != nullptr);
		LVK_ASSERT(supports_graphics_interop());

		try_attach_graphics_interop_context();

#ifdef _WIN32
		// DirectX11 Interop
		auto texture = static_cast<ID3D11Texture2D*>(gs_texture_get_obj(src));

		// Pre-validate texture format
		D3D11_TEXTURE2D_DESC desc = {0};
		texture->GetDesc(&desc);
		LVK_ASSERT(cv::directx::getTypeFromDXGI_FORMAT(desc.Format) >= 0);

		cv::directx::convertFromD3D11Texture2D(texture, dst);
#else  
		// OpenGL Interop
		
		// Pre-validate texture format
		const auto format = gs_texture_get_color_format(src);
		LVK_ASSERT(format == GS_RGBA || format == GS_RGBA_UNORM);

		cv::ogl::Texture2D texture(
			gs_texture_get_height(src),
			gs_texture_get_width(src),
			cv::ogl::Texture2D::RGBA,
			*static_cast<uint32_t*>(gs_texture_get_obj(src)),
			false
		);

		cv::ogl::convertFromGLTexture2D(texture, dst);
#endif
	}

//---------------------------------------------------------------------------------------------------------------------

	void export_texture(cv::UMat& src, gs_texture_t* dst)
	{
		LVK_ASSERT(dst != nullptr);
		LVK_ASSERT(src.cols == gs_texture_get_width(dst));
		LVK_ASSERT(src.rows == gs_texture_get_height(dst));
		LVK_ASSERT(supports_graphics_interop());

		try_attach_graphics_interop_context();

#ifdef _WIN32 
		// DirectX11 Interop
		auto texture = static_cast<ID3D11Texture2D*>(gs_texture_get_obj(dst));

		// Pre-validate texture format
		D3D11_TEXTURE2D_DESC desc = {0};
		texture->GetDesc(&desc);
		LVK_ASSERT(src.type() == cv::directx::getTypeFromDXGI_FORMAT(desc.Format));

		cv::directx::convertToD3D11Texture2D(src, texture);
#else 
		// OpenGL Interop

		// Pre-validate texture format
		const auto format = gs_texture_get_color_format(dst);
		LVK_ASSERT(format == GS_RGBA || format == GS_RGBA_UNORM);

		cv::ogl::Texture2D texture(
			gs_texture_get_height(dst),
			gs_texture_get_width(dst),
			cv::ogl::Texture2D::RGBA,
			*static_cast<uint32_t*>(gs_texture_get_obj(dst)),
			false
		);

		cv::ogl::convertToGLTexture2D(src, texture);
#endif
	}

//---------------------------------------------------------------------------------------------------------------------

}
