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

#include "../../Interop/FrameIngest.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

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

	ADBFilter* ADBFilter::Create(obs_source_t* context, obs_data_t* settings)
	{
		LVK_ASSERT(context != nullptr && settings != nullptr);

		auto filter = new ADBFilter(context);

		if(!filter->validate())
		{
			delete filter;
			return nullptr;
		}

		filter->configure(settings);

		return filter;
	}

//---------------------------------------------------------------------------------------------------------------------

	ADBFilter::ADBFilter(obs_source_t* context)
		: VisionFilter(context),
		  m_Context(context),
		  m_TestMode(false),
		  m_DetectionLevels(0),
		  m_BlockGrid(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_ChannelMask(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_BlockMask(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_Buffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_DeblockBuffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_FloatBuffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_KeepBlendMap(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_DeblockBlendMap(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY)
	{
		LVK_ASSERT(context != nullptr);
	}

//---------------------------------------------------------------------------------------------------------------------

	void ADBFilter::configure(obs_data_t* settings)
	{
		LVK_ASSERT(settings != nullptr);

		m_TestMode = obs_data_get_bool(settings, PROP_TEST_MODE);
		m_DetectionLevels = std::max<uint32_t>(obs_data_get_int(settings, PROP_STRENGTH), 1);
	}

//---------------------------------------------------------------------------------------------------------------------

	void ADBFilter::filter(cv::UMat& frame)
	{
		const auto start_time = os_gettime_ns();

		cv::cvtColor(frame, frame, cv::COLOR_YUV2BGR);

		// NOTE: De-blocking is achieved by adaptively blending a median filtered
		// frame with the original frame. Filtering is performed on a downscaled
		// frame to boost its performance and effective area. Blend maps are made
		// by comparing the original frame with a maximally blocked version of the
		// frame. Blocks which are similar are assumed to either contain blocking
		// artifacts or have low enough detail to not be affected by the smoothing.
		// Blocks whose difference surpasses the threshold across all color channels
		// are assumed to contain detail that should not be smoothed. To make the
		// choice of threshold less strict; multiple thresholds are used, each with
		// their own weighting that increases as details become stronger.


		constexpr int macro_block_size = 16;
		const cv::Size block_grid_size = frame.size() / macro_block_size;

		cv::resize(frame, m_BlockGrid, block_grid_size, 0, 0, cv::INTER_AREA);
		cv::resize(m_BlockGrid, m_Buffer, frame.size(), 0, 0, cv::INTER_NEAREST);
		cv::absdiff(frame, m_Buffer, m_Buffer);
		cv::resize(m_Buffer, m_BlockGrid, block_grid_size, 0, 0, cv::INTER_AREA);

		m_FloatBuffer.create(m_BlockGrid.size(), CV_32FC1);
		m_FloatBuffer.setTo(cv::Scalar(0.0));

		const float level_step = 1.0/m_DetectionLevels;
		for(uint32_t l = 0; l < m_DetectionLevels; l++)
		{
			cv::threshold(m_BlockGrid, m_ChannelMask, l, 255, cv::THRESH_BINARY);
			cv::inRange(m_ChannelMask, cv::Scalar::all(255), cv::Scalar::all(255), m_BlockMask);
			m_FloatBuffer.setTo(cv::Scalar((l + 1) * level_step),  m_BlockMask);
		}

		cv::resize(m_FloatBuffer, m_KeepBlendMap, frame.size(), 0, 0, cv::INTER_LINEAR);
		cv::absdiff(m_KeepBlendMap, cv::Scalar(1.0), m_DeblockBlendMap);

		const cv::Size filter_resolution(480, 270);
		cv::resize(frame, m_DeblockBuffer, filter_resolution, 0, 0, cv::INTER_AREA);
		cv::medianBlur(m_DeblockBuffer, m_DeblockBuffer, 5);
		cv::resize(m_DeblockBuffer, m_Buffer, frame.size(), 0, 0, cv::INTER_LINEAR);

		if(m_TestMode)
			m_Buffer.setTo(draw::BGR_MAGENTA);

		cv::blendLinear(frame, m_Buffer, m_KeepBlendMap, m_DeblockBlendMap, frame);

		cv::cvtColor(frame, frame, cv::COLOR_BGR2YUV);

		const auto end_time = os_gettime_ns();

		if(m_TestMode)
			draw_debug_info(frame, end_time - start_time);
	}

//---------------------------------------------------------------------------------------------------------------------

	void ADBFilter::draw_debug_info(cv::UMat& frame, const uint64_t frame_time_ns)
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
