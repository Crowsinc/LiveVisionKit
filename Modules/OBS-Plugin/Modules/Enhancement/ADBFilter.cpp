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
#include <functional>

#include "Utility/Locale.hpp"

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
	constexpr auto TIMING_SAMPLES = 30;

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

		const auto strength = obs_data_get_int(settings, PROP_STRENGTH);
		m_TestMode = obs_data_get_bool(settings, PROP_TEST_MODE);

		m_Filter.reconfigure([&](DeblockingSettings& settings) {
			settings.detection_levels = std::max<uint32_t>(strength, 1);
		});
	}

//---------------------------------------------------------------------------------------------------------------------

	ADBFilter::ADBFilter(obs_source_t* context)
		: VisionFilter(context),
		  m_Context(context),
		  m_Filter(),
		  m_FrameTimer(TIMING_SAMPLES)
	{
		LVK_ASSERT(context != nullptr);
	}

//---------------------------------------------------------------------------------------------------------------------

	void ADBFilter::filter(FrameBuffer& frame)
	{
		//TODO: replace this with FrameBuffer once that is converted
		if(m_TestMode)
		{
			m_Filter.profile(frame, frame, m_FrameTimer, true);
			draw_debug_hud(frame.data);
		}
		else m_Filter.process(frame, frame);
	}

//---------------------------------------------------------------------------------------------------------------------

	void ADBFilter::draw_debug_hud(cv::UMat& frame)
	{
		const auto frame_time_ms = m_FrameTimer.average().milliseconds();
		const auto deviation_ms = m_FrameTimer.deviation().milliseconds();

		draw::text(
			frame,
            cv::format("%.2fms (%.2fms)", frame_time_ms, deviation_ms),
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
