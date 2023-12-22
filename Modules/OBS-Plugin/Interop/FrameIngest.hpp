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

#pragma once

#include <memory>
#include <obs-module.h>
#include <LiveVisionKit.hpp>
#include <opencv2/opencv.hpp>

namespace lvk
{

	class FrameIngest
	{
	public:

        // TODO: make this extendable.
		static std::unique_ptr<FrameIngest> Select(video_format format);

		virtual ~FrameIngest() = default;

		void upload_obs_frame(const obs_source_frame* src, VideoFrame& dst);

		void download_ocl_frame(const VideoFrame& src, obs_source_frame* dst);

        VideoFrame::Format ocl_format() const;

        video_format obs_format() const;

    protected:

        FrameIngest(const video_format obs_format, const VideoFrame::Format ocl_format);

        virtual void to_ocl(const obs_source_frame* src, VideoFrame& dst) = 0;

        virtual void to_obs(const VideoFrame& src, obs_source_frame* dst) = 0;

	protected:

        static bool test_obs_frame(const obs_source_frame* frame);


        static void split_planes(const cv::UMat& src, cv::UMat& p1, cv::UMat& p2);

        static void split_planes(const cv::UMat& src, cv::UMat& p1, cv::UMat& p2, cv::UMat& p3);


        static void merge_planes(const cv::UMat& p1, const cv::UMat& p2, cv::UMat& dst);

        static void merge_planes(const cv::UMat& p1, const cv::UMat& p2, const cv::UMat& p3, cv::UMat& dst);


        static void fill_plane(obs_source_frame& dst, const uint32_t plane, const uint8_t value);


        void download_planes(
            const cv::UMat& plane_0,
            obs_source_frame& dst
        );

        void download_planes(
            const cv::UMat& plane_0,
            const cv::UMat& plane_1,
            obs_source_frame& dst
        );

        void download_planes(
            const cv::UMat& plane_0,
            const cv::UMat& plane_1,
            const cv::UMat& plane_2,
            obs_source_frame& dst
        );


        // NOTE: returns ROI to internal buffers
		cv::UMat upload_planes(
			const obs_source_frame& src,
			const uint32_t channels
		);

		// NOTE: returns ROI to internal buffers
		cv::UMat upload_planes(
			const obs_source_frame& src,
			const cv::Size& plane_0_size,
			const uint32_t plane_0_channels
		);
		
		// NOTE: returns ROI to internal buffers
		std::tuple<cv::UMat, cv::UMat> upload_planes(
			const obs_source_frame& src,
			const cv::Size& plane_0_size,
			const uint32_t plane_0_channels,
			const cv::Size& plane_1_size,
			const uint32_t plane_1_channels
		);
		
		// NOTE: returns ROI to internal buffers
		std::tuple<cv::UMat, cv::UMat, cv::UMat> upload_planes(
			const obs_source_frame& src,
			const cv::Size& plane_0_size,
			const uint32_t plane_0_channels,
			const cv::Size& plane_1_size,
			const uint32_t plane_1_channels,
			const cv::Size& plane_2_size,
			const uint32_t plane_2_channels
		);

	private:
		video_format m_OBSFormat = VIDEO_FORMAT_NONE;
	    VideoFrame::Format m_OCLFormat = VideoFrame::UNKNOWN;

        VideoFrame m_FormatConversionBuffer;
		cv::UMat m_ImportBuffer{cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY};
		cv::UMat m_ExportBuffer{cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY};
	};


	// Planar 4xx formats
	class I4XXIngest : public FrameIngest
	{
	public:

		explicit I4XXIngest(video_format i4xx_format);

    protected:

		void to_ocl(const obs_source_frame* src, VideoFrame& dst) override;
	
		void to_obs(const VideoFrame& src, obs_source_frame* dst) override;

	private:
		cv::Size2f m_ChromaScaling;
		
		// NOTE: We assume this will automatically initialize on the GPU
		cv::UMat m_YPlane, m_UPlane, m_VPlane;
		cv::UMat m_USubPlane, m_VSubPlane;
	};
	
	// Semi-planar NV12 format
	class NV12Ingest : public FrameIngest
	{
	public:

		NV12Ingest();

    protected:

        void to_ocl(const obs_source_frame* src, VideoFrame& dst) override;
		
		void to_obs(const VideoFrame& src, obs_source_frame* dst) override;

	private:
		// NOTE: We assume this will automatically initialize on the GPU
		cv::UMat m_YPlane, m_UVPlane, m_UVSubPlane;
	};

	// Packed 422 formats
	class P422Ingest : public FrameIngest
	{
	public:

		explicit P422Ingest(video_format packed_422_format);

    protected:

        void to_ocl(const obs_source_frame* src, VideoFrame& dst) override;

        void to_obs(const VideoFrame& src, obs_source_frame* dst) override;
	
	private:
		bool m_YFirst, m_UFirst;
		
		// NOTE: We assume this will automatically initialize on the GPU
		cv::UMat m_YPlane, m_UVPlane, m_UVSubPlane, m_MixBuffer;
	};

	// Packed 444 formats
	class P444Ingest : public FrameIngest
	{
	public:

		P444Ingest();

    protected:

        void to_ocl(const obs_source_frame* src, VideoFrame& dst) override;

        void to_obs(const VideoFrame& src, obs_source_frame* dst) override;
    private:
        // NOTE: We assume this will automatically initialize on the GPU
        cv::UMat m_MixBuffer;
	};

	// Uncompressed formats
	class DirectIngest : public FrameIngest
	{
	public:

		explicit DirectIngest(const video_format uncompressed_format);

    protected:

        void to_ocl(const obs_source_frame* src, VideoFrame& dst) override;

        void to_obs(const VideoFrame& src, obs_source_frame* dst) override;

    private:

        static VideoFrame::Format match_obs_format(const video_format obs_format);
	};

}

