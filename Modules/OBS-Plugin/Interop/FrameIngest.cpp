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
#include <LiveVisionKit.hpp>

#include "Utility/ScopedProfiler.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

    // NOTE: The maximmum size is to avoid any possibility of integer overflow when uploading the
    // textures to UMats, whose sizing is specified in 32bit integers. This is most relevant when
    // uploading the frames, as they are uploaded in bulk as 1 dimensionsl buffers.
    constexpr auto MAX_TEXTURE_SIZE = 8192;

//---------------------------------------------------------------------------------------------------------------------

	std::unique_ptr<FrameIngest> FrameIngest::Select(video_format format)
	{
		switch(format)
		{
			// Planar 4xx formats
			case video_format::VIDEO_FORMAT_YUVA:
			case video_format::VIDEO_FORMAT_I444:
			case video_format::VIDEO_FORMAT_I42A:
			case video_format::VIDEO_FORMAT_I422:
			case video_format::VIDEO_FORMAT_I40A:
			case video_format::VIDEO_FORMAT_I420:
				return std::make_unique<I4XXIngest>(format);

			// Semi-planar NV12 format
			case video_format::VIDEO_FORMAT_NV12:
				return std::make_unique<NV12Ingest>();

			// Packed 42x formats
			case video_format::VIDEO_FORMAT_YVYU:
			case video_format::VIDEO_FORMAT_YUY2:
			case video_format::VIDEO_FORMAT_UYVY:
				return std::make_unique<P422Ingest>(format);

			// Packed 444 YUV formats
			case video_format::VIDEO_FORMAT_AYUV:
				return std::make_unique<P444Ingest>();

			// Packed uncompressed non-YUV formats
			case video_format::VIDEO_FORMAT_Y800:
			case video_format::VIDEO_FORMAT_RGBA:
			case video_format::VIDEO_FORMAT_BGRX:
			case video_format::VIDEO_FORMAT_BGRA:
			case video_format::VIDEO_FORMAT_BGR3:
				return std::make_unique<DirectIngest>(format);

			// Unsupported formats
			default:
				return nullptr;
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	FrameIngest::FrameIngest(const video_format obs_format, const VideoFrame::Format ocl_format)
		: m_OBSFormat(obs_format),
          m_OCLFormat(ocl_format)
	{
        LVK_ASSERT(m_OBSFormat != VIDEO_FORMAT_NONE);
        LVK_ASSERT(m_OCLFormat != VideoFrame::UNKNOWN);
	}

//---------------------------------------------------------------------------------------------------------------------

    void FrameIngest::upload_obs_frame(const obs_source_frame* src, VideoFrame& dst)
    {
        LVK_ASSERT(test_obs_frame(src) && src->format == m_OBSFormat);

        to_ocl(src, dst);

        // Update Metadata.
        dst.timestamp = src->timestamp;
        dst.format = m_OCLFormat;
    }

//---------------------------------------------------------------------------------------------------------------------

    void FrameIngest::download_ocl_frame(const VideoFrame& src, obs_source_frame* dst)
    {
        LVK_ASSERT(test_obs_frame(dst) && dst->format == m_OBSFormat);
        LVK_ASSERT(src.has_known_format());

        // Attempt to convert the source to the expected format before downloading.
        src.viewAsFormat(m_FormatConversionBuffer, m_OCLFormat);
        to_obs(m_FormatConversionBuffer, dst);

        // Update Metadata.
        dst->timestamp = src.timestamp;
    }

//---------------------------------------------------------------------------------------------------------------------

    VideoFrame::Format FrameIngest::ocl_format() const
    {
        return m_OCLFormat;
    }

//---------------------------------------------------------------------------------------------------------------------

	video_format FrameIngest::obs_format() const
	{
		return m_OBSFormat;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool FrameIngest::test_obs_frame(const obs_source_frame* frame)
	{
		return frame != nullptr
			&& frame->data[0] != nullptr
			&& frame->width > 0
			&& frame->height > 0
			&& frame->linesize[0] >= frame->width
			&& frame->format != VIDEO_FORMAT_NONE;
	}

//---------------------------------------------------------------------------------------------------------------------

    void FrameIngest::split_planes(const cv::UMat& src, cv::UMat& p1, cv::UMat& p2, cv::UMat& p3)
    {
        LVK_ASSERT(src.type() == CV_8UC3);
        LVK_ASSERT(!src.empty());

        p1.create(src.size(), CV_8UC1, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
        p2.create(src.size(), CV_8UC1, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
        p3.create(src.size(), CV_8UC1, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

        cv::split(src, std::vector<cv::UMat>{p1, p2, p3});
    }

//---------------------------------------------------------------------------------------------------------------------

    void FrameIngest::split_planes(const cv::UMat& src, cv::UMat& p1, cv::UMat& p2)
    {
        LVK_ASSERT(!src.empty());
        LVK_ASSERT(src.type() == CV_8UC2);

        p1.create(src.size(), CV_8UC1, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
        p2.create(src.size(), CV_8UC1, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

        cv::split(src, std::vector<cv::UMat>{p1, p2});
    }

//---------------------------------------------------------------------------------------------------------------------

	void FrameIngest::merge_planes(const cv::UMat& p1, const cv::UMat& p2, const cv::UMat& p3, cv::UMat& dst)
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

	void FrameIngest::merge_planes(const cv::UMat& p1, const cv::UMat& p2, cv::UMat& dst)
	{
		LVK_ASSERT(p1.type() == CV_8UC1);
		LVK_ASSERT(p2.type() == CV_8UC1);
		LVK_ASSERT(!p1.empty());
		LVK_ASSERT(!p2.empty());

		cv::merge(std::vector<cv::UMat>{p1, p2}, dst);
	}

//---------------------------------------------------------------------------------------------------------------------

    void FrameIngest::fill_plane(obs_source_frame& dst, const uint32_t plane, const uint8_t value)
    {
        LVK_ASSERT(plane < MAX_AV_PLANES);
        LVK_ASSERT(test_obs_frame(&dst));
        LVK_ASSERT(dst.data[plane] != nullptr);

        std::memset(dst.data[plane], value, static_cast<size_t>(dst.width) * dst.height);
    }

//---------------------------------------------------------------------------------------------------------------------

	// NOTE: returns ROI to internal buffers
	cv::UMat FrameIngest::upload_planes(
		const obs_source_frame& src,
		const uint32_t channels
	)
	{
		return upload_planes(
            src,
            cv::Size(
                static_cast<int>(src.width),
                static_cast<int>(src.height)
            ),
            channels
        );
	}

//---------------------------------------------------------------------------------------------------------------------

    void FrameIngest::download_planes(
        const cv::UMat& plane_0,
        obs_source_frame& dst,
        uint64_t plane_offset
    )
    {
        LVK_ASSERT(!plane_0.empty());
        LVK_ASSERT(dst.data[0] != nullptr);
        LVK_ASSERT(dst.width <= MAX_TEXTURE_SIZE);
        LVK_ASSERT(dst.height <= MAX_TEXTURE_SIZE);
        LVK_ASSERT_RANGE(plane_0.cols, 1, dst.width);
        LVK_ASSERT_RANGE(plane_0.rows, 1, dst.height);
        LVK_PROFILE;

        const int export_length = static_cast<int>(plane_0.total() * plane_0.elemSize());
        const int export_offset = static_cast<int>(dst.data[plane_offset] - dst.data[0]);

        plane_0.reshape(1, 1).copyTo(cv::Mat(export_offset, export_length, CV_8UC1, dst.data[0]));
    }

//---------------------------------------------------------------------------------------------------------------------

    void FrameIngest::download_planes(
        const cv::UMat& plane_0,
        const cv::UMat& plane_1,
        obs_source_frame& dst
    )
    {
        LVK_ASSERT(!plane_0.empty())
        LVK_ASSERT(!plane_1.empty());
        LVK_ASSERT(dst.data[0] != nullptr);
        LVK_ASSERT(dst.data[1] != nullptr);
        LVK_ASSERT(dst.width <= MAX_TEXTURE_SIZE);
        LVK_ASSERT(dst.height <= MAX_TEXTURE_SIZE);
        LVK_ASSERT_RANGE(plane_0.cols, 1, dst.width);
        LVK_ASSERT_RANGE(plane_0.rows, 1, dst.height);
        LVK_ASSERT_RANGE(plane_1.cols, 1, dst.width);
        LVK_ASSERT_RANGE(plane_1.rows, 1, dst.height);
        LVK_PROFILE;

        // NOTE: Downloads are done in bulk by utilising the fact that the OBS planes
        // are all stored in one contiguous span of memory starting at src.data[0].
        // We must conserve the padding which exists between planes in memory.

        const int plane_0_length = static_cast<int>(plane_0.total() * plane_0.elemSize());
        const int plane_1_length = static_cast<int>(plane_1.total() * plane_1.elemSize());

        const int plane_1_offset = static_cast<int>(dst.data[1] - dst.data[0]);
        const int export_length = plane_1_offset + plane_1_length;

        m_ExportBuffer.create(1, export_length, CV_8UC1);

        plane_0.reshape(1, 1).copyTo(m_ExportBuffer.colRange(0, plane_0_length));
        plane_1.reshape(1, 1).copyTo(m_ExportBuffer.colRange(plane_1_offset, plane_1_offset + plane_1_length));

        m_ExportBuffer.copyTo(cv::Mat(1, export_length, CV_8UC1, dst.data[0]));
    }

//---------------------------------------------------------------------------------------------------------------------

    void FrameIngest::download_planes(
        const cv::UMat& plane_0,
        const cv::UMat& plane_1,
        const cv::UMat& plane_2,
        obs_source_frame& dst
    )
    {
        LVK_ASSERT(!plane_0.empty());
        LVK_ASSERT(!plane_1.empty());
        LVK_ASSERT(!plane_2.empty());
        LVK_ASSERT(dst.data[0] != nullptr);
        LVK_ASSERT(dst.data[1] != nullptr);
        LVK_ASSERT(dst.data[2] != nullptr);
        LVK_ASSERT(dst.width <= MAX_TEXTURE_SIZE);
        LVK_ASSERT(dst.height <= MAX_TEXTURE_SIZE);
        LVK_ASSERT_RANGE(plane_0.cols, 1, dst.width);
        LVK_ASSERT_RANGE(plane_0.rows, 1, dst.height);
        LVK_ASSERT_RANGE(plane_1.cols, 1, dst.width);
        LVK_ASSERT_RANGE(plane_1.rows, 1, dst.height);
        LVK_ASSERT_RANGE(plane_2.cols, 1, dst.width)
        LVK_ASSERT_RANGE(plane_2.rows, 1, dst.height);
        LVK_PROFILE;

        // NOTE: Downloads are done in bulk by utilising the fact that the OBS planes
        // are all stored in one contiguous span of memory starting at src.data[0].
        // We must conserve the padding which exists between planes in memory.

        const int plane_0_length = static_cast<int>(plane_0.total() * plane_0.elemSize());
        const int plane_1_length = static_cast<int>(plane_1.total() * plane_1.elemSize());
        const int plane_2_length = static_cast<int>(plane_2.total() * plane_2.elemSize());

        const int plane_1_offset = static_cast<int>(dst.data[1] - dst.data[0]);
        const int plane_2_offset = static_cast<int>(dst.data[2] - dst.data[0]);
        const int export_length = plane_2_offset + plane_2_length;

        m_ExportBuffer.create(1, export_length, CV_8UC1);

        plane_0.reshape(1, 1).copyTo(m_ExportBuffer.colRange(0, plane_0_length));
        plane_1.reshape(1, 1).copyTo(m_ExportBuffer.colRange(plane_1_offset, plane_1_offset + plane_1_length));
        plane_2.reshape(1, 1).copyTo(m_ExportBuffer.colRange(plane_2_offset, plane_2_offset + plane_2_length));

        m_ExportBuffer.copyTo(cv::Mat(1, export_length, CV_8UC1, dst.data[0]));
    }

//---------------------------------------------------------------------------------------------------------------------

	// NOTE: returns ROI to internal buffers
	cv::UMat FrameIngest::upload_planes(
		const obs_source_frame& src,
		const cv::Size& plane_0_size,
		const uint32_t plane_0_channels
	)
	{
        LVK_ASSERT(src.data[0] != nullptr);
        LVK_ASSERT(src.width <= MAX_TEXTURE_SIZE);
        LVK_ASSERT(src.height <= MAX_TEXTURE_SIZE);
		LVK_ASSERT_RANGE(plane_0_size.height, 1, src.height);
		LVK_ASSERT_RANGE(plane_0_size.width, 1, src.width);
		LVK_ASSERT_RANGE(plane_0_channels, 1, 4);
        LVK_PROFILE;

		const int import_length = plane_0_size.area() * static_cast<int>(plane_0_channels);

		cv::Mat(
            1,
            import_length,
            CV_8UC1,
            src.data[0]
        ).copyTo(m_ImportBuffer);

		return m_ImportBuffer.reshape(
            static_cast<int>(plane_0_channels),
            plane_0_size.height
        );
	}

//---------------------------------------------------------------------------------------------------------------------

	// NOTE: returns ROI to internal buffers
	std::tuple<cv::UMat, cv::UMat> FrameIngest::upload_planes(
		const obs_source_frame& src,
		const cv::Size& plane_0_size,
		const uint32_t plane_0_channels,
		const cv::Size& plane_1_size,
		const uint32_t plane_1_channels
	)
	{
		LVK_ASSERT(src.data[0] != nullptr);
		LVK_ASSERT(src.data[1] != nullptr);
        LVK_ASSERT(src.width <= MAX_TEXTURE_SIZE);
        LVK_ASSERT(src.height <= MAX_TEXTURE_SIZE);
		LVK_ASSERT_RANGE(plane_0_size.height, 1, src.height);
		LVK_ASSERT_RANGE(plane_0_size.width, 1, src.width);
		LVK_ASSERT_RANGE(plane_0_channels, 1, 4);
		LVK_ASSERT_RANGE(plane_1_size.height, 1, src.height);
		LVK_ASSERT_RANGE(plane_1_size.width, 1, src.width);
		LVK_ASSERT_RANGE(plane_1_channels, 1, 4);
        LVK_PROFILE;

		// NOTE: Uploads are done in bulk by utilising the fact that the OBS planes
		// are all stored in one contiguous span of memory starting at src.data[0].
		// Padding exists between planes which must be avoided.

		const int plane_0_length = plane_0_size.area() * static_cast<int>(plane_0_channels);
		const int plane_1_length = plane_1_size.area() * static_cast<int>(plane_1_channels);

		const int plane_1_offset = static_cast<int>(src.data[1] - src.data[0]);
		const int import_length = plane_1_offset + plane_1_length;

		cv::Mat(
            1,
            import_length,
            CV_8UC1,
            src.data[0]
        ).copyTo(m_ImportBuffer);

		return std::make_tuple<cv::UMat, cv::UMat>(
			m_ImportBuffer.colRange(0, plane_0_length).reshape(
                static_cast<int>(plane_0_channels),
                plane_0_size.height
            ),
			m_ImportBuffer.colRange(plane_1_offset, plane_1_offset + plane_1_length).reshape(
                static_cast<int>(plane_1_channels),
                plane_1_size.height
            )
		);
	}

//---------------------------------------------------------------------------------------------------------------------
	
	// NOTE: returns ROI to internal buffers
	std::tuple<cv::UMat, cv::UMat, cv::UMat> FrameIngest::upload_planes(
		const obs_source_frame& src,
		const cv::Size& plane_0_size,
		const uint32_t plane_0_channels,
		const cv::Size& plane_1_size,
		const uint32_t plane_1_channels,
		const cv::Size& plane_2_size,
		const uint32_t plane_2_channels
	)
	{
		LVK_ASSERT(src.data[0] != nullptr);
		LVK_ASSERT(src.data[1] != nullptr);
		LVK_ASSERT(src.data[2] != nullptr);
        LVK_ASSERT(src.width <= MAX_TEXTURE_SIZE);
        LVK_ASSERT(src.height <= MAX_TEXTURE_SIZE);
        LVK_ASSERT_RANGE(plane_0_size.height, 1, src.height);
        LVK_ASSERT_RANGE(plane_0_size.width, 1, src.width);
        LVK_ASSERT_RANGE(plane_0_channels, 1, 4);
        LVK_ASSERT_RANGE(plane_1_size.height, 1, src.height);
        LVK_ASSERT_RANGE(plane_1_size.width, 1, src.width);
        LVK_ASSERT_RANGE(plane_1_channels, 1, 4);
		LVK_ASSERT_RANGE(plane_2_size.height, 1, src.height);
		LVK_ASSERT_RANGE(plane_2_size.width, 1, src.width);
		LVK_ASSERT_RANGE(plane_2_channels, 1, 4);
        LVK_PROFILE;

		// NOTE: Uploads are done in bulk by utilising the fact that the OBS planes
		// are all stored in one contiguous span of memory starting at src.data[0].
		// Padding exists between planes which must be avoided.

		const int plane_0_length = plane_0_size.area() * static_cast<int>(plane_0_channels);
		const int plane_1_length = plane_1_size.area() * static_cast<int>(plane_1_channels);
		const int plane_2_length = plane_2_size.area() * static_cast<int>(plane_2_channels);

		const int plane_1_offset = static_cast<int>(src.data[1] - src.data[0]);
		const int plane_2_offset = static_cast<int>(src.data[2] - src.data[0]);
		const int import_length = plane_2_offset + plane_2_length;

		cv::Mat(
            1,
            import_length,
            CV_8UC1,
            src.data[0]
        ).copyTo(m_ImportBuffer);

		return std::make_tuple(
			m_ImportBuffer.colRange(0, plane_0_length).reshape(
                static_cast<int>(plane_0_channels),
                plane_0_size.height
            ),
			m_ImportBuffer.colRange(plane_1_offset, plane_1_offset + plane_1_length).reshape(
                static_cast<int>(plane_1_channels),
                plane_1_size.height
            ),
			m_ImportBuffer.colRange(plane_2_offset, plane_2_offset + plane_2_length).reshape(
                static_cast<int>(plane_2_channels),
                plane_2_size.height
            )
		);
	}

//---------------------------------------------------------------------------------------------------------------------

	I4XXIngest::I4XXIngest(video_format i4xx_format)
		: FrameIngest(i4xx_format, VideoFrame::YUV),
		  m_ChromaScaling(
			 any_of(i4xx_format, VIDEO_FORMAT_YUVA, VIDEO_FORMAT_I444) ? 1.0f : 0.5f,
			 any_of(i4xx_format, VIDEO_FORMAT_I40A, VIDEO_FORMAT_I420) ? 0.5f : 1.0f
		  )
	{
		LVK_ASSERT(any_of(i4xx_format,
			VIDEO_FORMAT_YUVA, VIDEO_FORMAT_I444,
			VIDEO_FORMAT_I42A, VIDEO_FORMAT_I422,
			VIDEO_FORMAT_I40A, VIDEO_FORMAT_I420
		));
	}

//---------------------------------------------------------------------------------------------------------------------

	void I4XXIngest::to_ocl(const obs_source_frame* src, VideoFrame& dst)
	{
        LVK_PROFILE;

        auto& frame = *src;
		
		const cv::Size frame_size(static_cast<int>(src->width), static_cast<int>(src->height));
        const cv::Size chroma_size = m_ChromaScaling * cv::Size2f(frame_size);

		auto [y_roi, u_roi, v_roi] = upload_planes(
			frame,
			frame_size, 1,
			chroma_size, 1,
			chroma_size, 1
		);

		LVK_ASSERT(!y_roi.empty());
		LVK_ASSERT(!u_roi.empty());
		LVK_ASSERT(!v_roi.empty());

		if(chroma_size != frame_size)
		{
			cv::resize(u_roi, m_USubPlane, frame_size, 0, 0, cv::INTER_LINEAR);
			cv::resize(v_roi, m_VSubPlane, frame_size, 0, 0, cv::INTER_LINEAR);

			merge_planes(y_roi, m_USubPlane, m_VSubPlane, dst);
		}
		else merge_planes(y_roi, u_roi, v_roi, dst);
	}

//---------------------------------------------------------------------------------------------------------------------

	void I4XXIngest::to_obs(const VideoFrame &src, obs_source_frame* dst)
    {
        LVK_PROFILE;

        auto& frame = *dst;

		split_planes(src, m_YPlane, m_UPlane, m_VPlane);

		if(m_ChromaScaling.width != 1.0f || m_ChromaScaling.height != 1.0f)
		{
			cv::resize(
				m_UPlane,
				m_USubPlane,
				cv::Size(),
				m_ChromaScaling.width,
				m_ChromaScaling.height,
				cv::INTER_AREA
			);

			cv::resize(
				m_VPlane,
				m_VSubPlane,
				cv::Size(),
				m_ChromaScaling.width,
				m_ChromaScaling.height,
				cv::INTER_AREA
			);

			download_planes(m_YPlane, m_USubPlane, m_VSubPlane, frame);
		}
		else download_planes(m_YPlane, m_UPlane, m_VPlane, frame);
	}

//---------------------------------------------------------------------------------------------------------------------

	NV12Ingest::NV12Ingest()
		: FrameIngest(VIDEO_FORMAT_NV12, VideoFrame::YUV)
	{}

//---------------------------------------------------------------------------------------------------------------------

	void NV12Ingest::to_ocl(const obs_source_frame* src, VideoFrame& dst) {
        LVK_PROFILE;

		auto& frame = *src;

		const cv::Size frame_size(static_cast<int>(frame.width), static_cast<int>(frame.height));
		const cv::Size chroma_size = frame_size / 2;

		auto [y_roi, uv_roi] = upload_planes(
			frame,
			frame_size, 1,
			chroma_size, 2
		);

		cv::resize(uv_roi, m_UVPlane, frame_size, 0, 0, cv::INTER_LINEAR);
		dst.create(frame_size, CV_8UC3, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		cv::mixChannels({{y_roi, m_UVPlane}}, std::vector<cv::UMat>{dst}, {0,0,  1,1,  2,2});
	}

//---------------------------------------------------------------------------------------------------------------------

	void NV12Ingest::to_obs(const VideoFrame& src, obs_source_frame* dst)
	{
        LVK_PROFILE;

		auto& frame = *dst;

		m_YPlane.create(src.size(), CV_8UC1);
		m_UVPlane.create(src.size(), CV_8UC2);

		cv::mixChannels({src}, std::vector<cv::UMat>{m_YPlane, m_UVPlane}, {0,0, 1,1,  2,2});

		cv::resize(m_UVPlane, m_UVSubPlane, cv::Size(), 0.5, 0.5, cv::INTER_AREA);

		download_planes(m_YPlane, m_UVSubPlane, frame);
	}

//---------------------------------------------------------------------------------------------------------------------

	P422Ingest::P422Ingest(video_format packed_422_format)
		: FrameIngest(packed_422_format, VideoFrame::YUV),
		  m_YFirst(packed_422_format != VIDEO_FORMAT_UYVY),
		  m_UFirst(packed_422_format != VIDEO_FORMAT_YVYU)
	{
		LVK_ASSERT(any_of(packed_422_format, VIDEO_FORMAT_YVYU, VIDEO_FORMAT_YUY2, VIDEO_FORMAT_UYVY));
	}
	
//---------------------------------------------------------------------------------------------------------------------

	void P422Ingest::to_ocl(const obs_source_frame* src, VideoFrame& dst)
	{
        LVK_PROFILE;

        auto& frame = *src;

		cv::UMat plane_roi = upload_planes(frame, 2);

		// Re-interpret uv plane as 2 components to remove interleaving, then upsample to correct size
		cv::extractChannel(plane_roi, m_UVSubPlane, m_YFirst ? 1 : 0);
		cv::resize(m_UVSubPlane.reshape(2, m_UVSubPlane.rows), m_UVPlane, plane_roi.size(), 0, 0, cv::INTER_LINEAR);

		std::vector<int> from_to(6);
		if(m_UFirst)
			from_to = {(m_YFirst ? 0 : 1),0,  2,1,  3,2};
		else
			from_to = {(m_YFirst ? 0 : 1),0,  2,2,  3,1};

		// Merge upsampled uv plane back with the y plane
		dst.create(plane_roi.size(), CV_8UC3, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		cv::mixChannels({{plane_roi, m_UVPlane}}, std::vector<cv::UMat>{dst}, from_to);
	}

//---------------------------------------------------------------------------------------------------------------------

	void P422Ingest::to_obs(const VideoFrame& src, obs_source_frame* dst)
	{
        LVK_PROFILE;

		auto& frame = *dst;

		m_MixBuffer.create(src.size(), CV_8UC2);

		// Extract uv planes
		if(m_UFirst)
			cv::mixChannels({src}, std::vector<cv::UMat>{m_MixBuffer}, {1,0,  2,1});
		else
			cv::mixChannels({src}, std::vector<cv::UMat>{m_MixBuffer}, {2,0,  1,1});

		// Subsample uv plane width and re-interpret as one channel to interleave u and v components
		cv::resize(m_MixBuffer, m_UVPlane, cv::Size(), 0.5, 1.0, cv::INTER_AREA);
		m_UVPlane = m_UVPlane.reshape(1, m_UVPlane.rows);

		// Pack y and interleaved uv planes
		cv::extractChannel(src, m_YPlane, 0);
		if(m_YFirst)
			cv::mixChannels({{m_YPlane, m_UVPlane}}, std::vector<cv::UMat>{m_MixBuffer}, {0,0,  1,1});
		else
			cv::mixChannels({{m_YPlane, m_UVPlane}}, std::vector<cv::UMat>{m_MixBuffer}, {0,1,  1,0});

		download_planes(m_MixBuffer, frame);
	}

//---------------------------------------------------------------------------------------------------------------------

	P444Ingest::P444Ingest()
		: FrameIngest(VIDEO_FORMAT_AYUV, VideoFrame::YUV)
	{}

//---------------------------------------------------------------------------------------------------------------------

	void P444Ingest::to_ocl(const obs_source_frame* src, VideoFrame& dst)
	{
        LVK_PROFILE;

		auto& frame = *src;

		cv::UMat plane_roi = upload_planes(frame, 4);

		dst.create(plane_roi.size(), CV_8UC3, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
		cv::mixChannels({plane_roi}, std::vector<cv::UMat>{dst}, {1,0,  2,1,  3,2});
	}

//---------------------------------------------------------------------------------------------------------------------

	void P444Ingest::to_obs(const VideoFrame& src, obs_source_frame* dst)
	{
        LVK_PROFILE;

		auto& frame = *dst;

        // Insert YUV planes after the A plane.
		download_planes(src, frame, 1);
	}

//---------------------------------------------------------------------------------------------------------------------
    DirectIngest::DirectIngest(const video_format uncompressed_format)
		: FrameIngest(uncompressed_format, match_obs_format(uncompressed_format))
	{
        LVK_ASSERT(uncompressed_format != VIDEO_FORMAT_NONE);
    }

//---------------------------------------------------------------------------------------------------------------------

    VideoFrame::Format DirectIngest::match_obs_format(const video_format obs_format)
    {
        switch(obs_format)
        {
            case video_format::VIDEO_FORMAT_Y800: return VideoFrame::GRAY;
            case video_format::VIDEO_FORMAT_RGBA: return VideoFrame::RGB;
            case video_format::VIDEO_FORMAT_BGRX:
            case video_format::VIDEO_FORMAT_BGRA:
            case video_format::VIDEO_FORMAT_BGR3: return VideoFrame::BGR;
            default: LVK_ASSERT(false && "format not supported");
        }
    }

//---------------------------------------------------------------------------------------------------------------------

	void DirectIngest::to_ocl(const obs_source_frame* src, VideoFrame& dst)
	{
        LVK_PROFILE;

        // NOTE: We treat 4-component formats such as BGRA as
        // 3-component instead in order to avoid unnecessarily
        // uploading the alpha plane, which is not used in LVK.
        upload_planes(
            *src,
            (obs_format() == VIDEO_FORMAT_Y800) ? 1 : 3
        ).copyTo(dst);
	}
	
//---------------------------------------------------------------------------------------------------------------------

	void DirectIngest::to_obs(const VideoFrame& src, obs_source_frame* dst)
	{
        LVK_PROFILE;

        // NOTE: All supported formats have the alpha plane as the final component.
        // Src formats have no alpha, so we directly upload into the first three components.
        download_planes(src, *dst);
	}
	
//---------------------------------------------------------------------------------------------------------------------

}