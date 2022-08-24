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

#include <util/platform.h>

#include "OBS/Utility/Locale.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	constexpr auto AREA_SCALING = 1.0/4;
	constexpr auto KERNEL_SIZE = 5;
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
			L("adb.strength"),
			STRENGTH_MIN,
			STRENGTH_MAX,
			1
		);

		obs_properties_add_bool(
			properties,
			PROP_TEST_MODE,
			L("f.testmode")
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
		  m_Context(context)
	{
		LVK_ASSERT(context != nullptr);
	}

//---------------------------------------------------------------------------------------------------------------------

	void ADBFilter::filter(cv::UMat& frame)
	{
		uint64_t start_time = os_gettime_ns();

		// NOTE: De-blocking is achieved by adaptively blending a median smoothed
		// frame with the original. Filtering occurs on a downscaled frame to boost
		// its performance and effective area. Blend maps are made by comparing the
		// original frame with a reference maximal blocking artifact frame, created
		// by simplifying each block to its average value. Not all frame resolutions
		// fit an integer number of macroblocks, so the frame must be padded or cropped.
		// Both these techniques lead to approximately the same result, so cropping
		// is preferred for performance. Blocks are assumed to be safe to smooth if
		// they are similar, by threshold,to the reference blocks. To make the choice
		// of threshold less strict for the user; multiple thresholds are used, each
		// with their own weighting that increases as details become stronger.

		const cv::Size macroblock_extent = frame.size() / MACROBLOCK_SIZE;
		const cv::Rect macroblock_region({0,0}, macroblock_extent * MACROBLOCK_SIZE);
		cv::UMat filter_ref = frame(macroblock_region);

		// Generate smooth frame
		cv::resize(filter_ref, m_DeblockBuffer, cv::Size(), AREA_SCALING, AREA_SCALING, cv::INTER_AREA);
		cv::medianBlur(m_DeblockBuffer, m_DeblockBuffer, KERNEL_SIZE);
		cv::resize(m_DeblockBuffer, m_SmoothFrame, macroblock_region.size(), 0, 0, cv::INTER_LINEAR);

		// Generate reference frame
		cv::extractChannel(filter_ref, m_DetectionFrame, 0);
		cv::resize(m_DetectionFrame, m_BlockGrid, macroblock_extent, 0, 0, cv::INTER_AREA);
		cv::resize(m_BlockGrid, m_ReferenceFrame, m_DetectionFrame.size(), 0, 0, cv::INTER_NEAREST);
		cv::absdiff(m_DetectionFrame, m_ReferenceFrame, m_DetectionFrame);
		cv::resize(m_DetectionFrame, m_BlockGrid, macroblock_extent, 0, 0, cv::INTER_AREA);

		// Produce blend maps
		m_FloatBuffer.create(macroblock_extent, CV_32FC1);
		m_FloatBuffer.setTo(cv::Scalar(0.0));

		const float level_step = 1.0/m_DetectionLevels;
		for(uint32_t l = 0; l < m_DetectionLevels; l++)
		{
			cv::threshold(m_BlockGrid, m_BlockMask, l, 255, cv::THRESH_BINARY);
			m_FloatBuffer.setTo(cv::Scalar((l + 1) * level_step),  m_BlockMask);
		}

		cv::resize(m_FloatBuffer, m_KeepBlendMap, filter_ref.size(), 0, 0, cv::INTER_LINEAR);
		cv::absdiff(m_KeepBlendMap, cv::Scalar(1.0), m_DeblockBlendMap);

		// Adaptively blend original and smooth frames

		if(m_TestMode)
			start_time += draw_debug_frame(m_SmoothFrame);

		cv::blendLinear(
			filter_ref,
			m_SmoothFrame,
			m_KeepBlendMap,
			m_DeblockBlendMap,
			filter_ref
		);

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

		frame.setTo(draw::YUV_MAGENTA);

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
