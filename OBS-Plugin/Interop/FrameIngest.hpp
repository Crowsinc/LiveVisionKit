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
#include <opencv2/opencv.hpp>

namespace lvk
{

	class FrameIngest
	{
	public:
		
		static std::unique_ptr<FrameIngest> Select(video_format format);

		virtual ~FrameIngest() = default;

		// Uploads the src obs_source_frame to the dst YUV UMat on the GPU
		virtual void upload(const obs_source_frame* src, cv::UMat& dst) = 0;

		// Downloads the src YUV UMat to the dst obs_source_frame, preserving any existing alpha channels
		virtual void download(const cv::UMat& src, obs_source_frame* dst) = 0;

		video_format format();

	protected:

		FrameIngest(video_format format);

		// NOTE: returns ROI to internal buffers
		cv::UMat upload_planes(
			const obs_source_frame& src,
			const uint32_t channels
		);

		// NOTE: returns ROI to internal buffers
		cv::UMat upload_planes(
			const obs_source_frame& src,
			const cv::Size plane_0_size,
			const uint32_t plane_0_channels
		);
		
		// NOTE: returns ROI to internal buffers
		std::tuple<cv::UMat, cv::UMat> upload_planes(
			const obs_source_frame& src,
			const cv::Size plane_0_size,
			const uint32_t plane_0_channels,
			const cv::Size plane_1_size,
			const uint32_t plane_1_channels
		);
		
		// NOTE: returns ROI to internal buffers
		std::tuple<cv::UMat, cv::UMat, cv::UMat> upload_planes(
			const obs_source_frame& src,
			const cv::Size plane_0_size,
			const uint32_t plane_0_channels,
			const cv::Size plane_1_size,
			const uint32_t plane_1_channels,
			const cv::Size plane_2_size,
			const uint32_t plane_2_channels
		);

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

		static void fill_plane(obs_source_frame& dst, const uint32_t plane, const uint8_t value);

		static void merge_planes(const cv::UMat& p1, const cv::UMat& p2, const cv::UMat& p3, cv::UMat& dst);

		static void merge_planes(const cv::UMat& p1, const cv::UMat& p2, cv::UMat& dst);

		static void split_planes(const cv::UMat& src, cv::UMat& p1, cv::UMat& p2, cv::UMat& p3);

		static void split_planes(const cv::UMat& src, cv::UMat& p1, cv::UMat& p2);

		static bool test_obs_frame(const obs_source_frame* frame);

	private:
		video_format m_Format = VIDEO_FORMAT_NONE;
	
		cv::UMat m_ImportBuffer{cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY};
		cv::UMat m_ExportBuffer{cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY};
	};


	// Planar 4xx formats
	class I4XXIngest : public FrameIngest
	{
	public:

		I4XXIngest(video_format i4xx_format);

		void upload(const obs_source_frame* src, cv::UMat& dst) override;
	
		void download(const cv::UMat& src, obs_source_frame* dst) override;

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

		virtual void upload(const obs_source_frame* src, cv::UMat& dst) override;
		
		virtual void download(const cv::UMat& src, obs_source_frame* dst) override;

	private:
		// NOTE: We assume this will automatically initialize on the GPU
		cv::UMat m_YPlane, m_UVPlane, m_UVSubPlane;
	};

	// Packed 422 formats
	class P422Ingest : public FrameIngest
	{
	public:

		P422Ingest(video_format packed_422_format);

		virtual void upload(const obs_source_frame* src, cv::UMat& dst) override;
		
		virtual void download(const cv::UMat& src, obs_source_frame* dst) override;
	
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

		virtual void upload(const obs_source_frame* src, cv::UMat& dst) override;
		
		virtual void download(const cv::UMat& src, obs_source_frame* dst) override;

	private:
		// NOTE: We assume this will automatically initialize on the GPU
		cv::UMat m_MixBuffer;
	};

	// Uncompressed formats
	class DirectIngest : public FrameIngest
	{
	public:

		DirectIngest(video_format uncompressed_format);

		virtual void upload(const obs_source_frame* src, cv::UMat& dst) override;

		virtual void download(const cv::UMat& src, obs_source_frame* dst) override;

	private:
		int m_Components;
		bool m_SteppedConversion;
		cv::ColorConversionCodes m_ForwardConversion, m_ForwardStepConversion;
		cv::ColorConversionCodes m_BackwardConversion, m_BackwardStepConversion;

		// NOTE: We assume this will automatically initialize on the GPU
		cv::UMat m_ConversionBuffer, m_StepConversionBuffer;
	};

}

