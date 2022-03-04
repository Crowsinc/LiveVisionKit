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

#include "ADBFilter.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	constexpr auto FILTER_AREA_SCALING = 1.0/4;
	constexpr auto MACROBLOCK_SIZE = 16;

	constexpr auto PROP_STRENGTH = "STRENGTH";
	constexpr auto STRENGTH_MAX = 5;
	constexpr auto STRENGTH_MIN = 1;
	constexpr auto STRENGTH_DEFAULT = 3;

	constexpr auto PROP_TEST_MODE = "TEST_MODE";
	constexpr auto TEST_MODE_DEFAULT = false;

	constexpr auto TIMING_THRESHOLD_MS = 3.0;

//---------------------------------------------------------------------------------------------------------------------

	obs_properties_t* ADBFilter::Properties()
	{
		obs_properties_t* properties = obs_properties_create();

		obs_properties_add_int_slider(
			properties,
			PROP_STRENGTH,
			"Strength",
			STRENGTH_MIN,
			STRENGTH_MAX,
			1
		);

		obs_properties_add_bool(
				properties,
				PROP_TEST_MODE,
				"Test Mode"
		);

		return properties;
	}

//---------------------------------------------------------------------------------------------------------------------

	void ADBFilter::LoadDefaults(obs_data_t* settings)
	{
		LVK_ASSERT(settings != nullptr);

		obs_data_set_default_int(settings, PROP_STRENGTH, STRENGTH_DEFAULT);
		obs_data_set_default_bool(settings, PROP_TEST_MODE, TEST_MODE_DEFAULT);
	}

//---------------------------------------------------------------------------------------------------------------------

	void ADBFilter::configure(obs_data_t* settings)
	{
		LVK_ASSERT(settings != nullptr);

		m_TestMode = obs_data_get_bool(settings, PROP_TEST_MODE);
		m_DetectionLevels = std::max<uint32_t>(obs_data_get_int(settings, PROP_STRENGTH), 1);
	}

//---------------------------------------------------------------------------------------------------------------------

	ADBFilter::ADBFilter(obs_source_t* context)
		: VisionFilter(context),
		  m_Context(context),
		  m_TestMode(false),
		  m_DetectionLevels(0),
		  m_PaddedFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_SmoothFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_BlockFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_BlockBuffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_DeblockBuffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_FloatBuffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_ChannelMask(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_BlockMask(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_KeepMap(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_DeblockMap(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY)
	{
		LVK_ASSERT(context != nullptr);
	}

//---------------------------------------------------------------------------------------------------------------------

	void ADBFilter::filter(cv::UMat& frame)
	{
		uint64_t start_time = os_gettime_ns();

		cv::cvtColor(frame, frame, cv::COLOR_YUV2BGR);

		// NOTE: De-blocking is achieved by adaptively blending a median smoothed
		// frame with the original. Filtering occurs on a downscaled frame to boost
		// its performance and effective area. Blend maps are made by comparing the
		// original frame with a reference blocking artifact frame, created by
		// simplifying each block to average color. Blocks are assumed to be safe
		// to smooth if it is similar, by threshold, to the reference across all color
		// channels. To make the threshold less strict; multiple thresholds are used,
		// each with their own weighting that increases as details become stronger.

		// Generate smooth frame
		cv::resize(frame, m_DeblockBuffer, cv::Size(), FILTER_AREA_SCALING, FILTER_AREA_SCALING, cv::INTER_AREA);
		cv::medianBlur(m_DeblockBuffer, m_DeblockBuffer, 5);
		cv::resize(m_DeblockBuffer, m_SmoothFrame, frame.size(), 0, 0, cv::INTER_LINEAR);

		// Generate reference frame
		const auto width_padding = (MACROBLOCK_SIZE - (frame.cols % MACROBLOCK_SIZE)) % MACROBLOCK_SIZE;
		const auto height_padding = (MACROBLOCK_SIZE - (frame.rows % MACROBLOCK_SIZE)) % MACROBLOCK_SIZE;
		const cv::Rect unpadded_region({0, 0}, frame.size());

		cv::copyMakeBorder(frame, m_PaddedFrame, 0, height_padding, 0, width_padding, cv::BORDER_CONSTANT);
		m_BlockBuffer.create(m_PaddedFrame.size() / MACROBLOCK_SIZE, CV_8UC3);

		cv::resize(m_PaddedFrame, m_BlockBuffer, m_BlockBuffer.size(), 0, 0, cv::INTER_AREA);
		cv::resize(m_BlockBuffer, m_BlockFrame, m_PaddedFrame.size(), 0, 0, cv::INTER_NEAREST);
		cv::absdiff(m_PaddedFrame, m_BlockFrame, m_PaddedFrame);
		cv::resize(m_PaddedFrame, m_BlockBuffer, m_BlockBuffer.size(), 0, 0, cv::INTER_AREA);

		// Produce blend maps
		m_FloatBuffer.create(m_BlockBuffer.size(), CV_32FC1);
		m_FloatBuffer.setTo(cv::Scalar(0.0));

		const float level_step = 1.0/m_DetectionLevels;
		for(uint32_t l = 0; l < m_DetectionLevels; l++)
		{
			cv::threshold(m_BlockBuffer, m_ChannelMask, l, 255, cv::THRESH_BINARY);
			cv::inRange(m_ChannelMask, cv::Scalar::all(255), cv::Scalar::all(255), m_BlockMask);
			m_FloatBuffer.setTo(cv::Scalar((l + 1) * level_step),  m_BlockMask);
		}

		cv::resize(m_FloatBuffer, m_KeepMap, m_PaddedFrame.size(), 0, 0, cv::INTER_LINEAR);
		cv::absdiff(m_KeepMap, cv::Scalar(1.0), m_DeblockMap);

		// Adaptively blend smooth and original frames

		if(m_TestMode)
			start_time += draw_debug_frame(m_SmoothFrame);

		cv::blendLinear(
			frame,
			m_SmoothFrame,
			m_KeepMap(unpadded_region),
			m_DeblockMap(unpadded_region),
			frame
		);

		cv::cvtColor(frame, frame, cv::COLOR_BGR2YUV);

		if(m_TestMode)
		{
			cv::ocl::finish();
			draw_debug_hud(frame, os_gettime_ns() - start_time);
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	uint64_t ADBFilter::draw_debug_frame(cv::UMat& frame)
	{
		cv::ocl::finish();
		const uint64_t start_time = os_gettime_ns();

		frame.setTo(draw::BGR_MAGENTA);

		cv::ocl::finish();
		return os_gettime_ns() - start_time;
	}

//---------------------------------------------------------------------------------------------------------------------

	void ADBFilter::draw_debug_hud(cv::UMat& frame, const uint64_t frame_time_ns)
	{
		const double frame_time_ms = frame_time_ns * 1.0e-6;


		//TODO: switch to C++20 fmt as soon as GCC supports it.
		std::stringstream time_text;
		time_text << std::fixed << std::setprecision(2);
		time_text << frame_time_ms << "ms";

		draw::text(
			frame,
			time_text.str(),
			cv::Point(5, 40),
			frame_time_ms < TIMING_THRESHOLD_MS ? draw::YUV_GREEN : draw::YUV_RED
		);
	}

//---------------------------------------------------------------------------------------------------------------------

	bool ADBFilter::validate() const
	{
		return m_Context != nullptr;
	}

//---------------------------------------------------------------------------------------------------------------------

}
