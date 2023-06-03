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

#include "Utility/ScopedProfiler.hpp"
#include "Utility/Locale.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	constexpr auto PROP_STRENGTH = "STRENGTH";
	constexpr auto PROP_STRENGTH_MAX = 5;
	constexpr auto PROP_STRENGTH_MIN = 1;
	constexpr auto PROP_STRENGTH_DEFAULT = 3;

	constexpr auto PROP_TEST_MODE = "TEST_MODE";
	constexpr auto PROP_TEST_MODE_DEFAULT = false;

	constexpr auto TIMING_THRESHOLD_MS = 3.0;
	constexpr auto TIMING_SAMPLES = 30;

//---------------------------------------------------------------------------------------------------------------------

	obs_properties_t* ADBFilter::Properties()
	{
		obs_properties_t* properties = obs_properties_create();

        // Strength Slider
		obs_properties_add_int_slider(
			properties,
			PROP_STRENGTH,
			L("adb.strength"),
			PROP_STRENGTH_MIN,
			PROP_STRENGTH_MAX,
			1
		);

        // Runtime Controls
        obs_properties_t* controls = obs_properties_create();
        obs_properties_add_group(
            properties,
            "CONTROL_GROUP",
            L("f.controls-group"),
            obs_group_type::OBS_GROUP_NORMAL,
            controls
        );

        // Test Mode Toggle
		obs_properties_add_bool(
            controls,
			PROP_TEST_MODE,
			L("f.testmode")
		);

		return properties;
	}

//---------------------------------------------------------------------------------------------------------------------

	void ADBFilter::LoadDefaults(obs_data_t* settings)
	{
		LVK_ASSERT(settings != nullptr);

		obs_data_set_default_int(settings, PROP_STRENGTH, PROP_STRENGTH_DEFAULT);
		obs_data_set_default_bool(settings, PROP_TEST_MODE, PROP_TEST_MODE_DEFAULT);
	}

//---------------------------------------------------------------------------------------------------------------------

	void ADBFilter::configure(obs_data_t* settings)
	{
		LVK_ASSERT(settings != nullptr);

		const auto strength = obs_data_get_int(settings, PROP_STRENGTH);
		m_TestMode = obs_data_get_bool(settings, PROP_TEST_MODE);

		m_Filter.reconfigure([&](DeblockingFilterSettings& settings) {
			settings.detection_levels = std::max<uint32_t>(strength, 1);
		});
	}

//---------------------------------------------------------------------------------------------------------------------

	ADBFilter::ADBFilter(obs_source_t* context)
		: VisionFilter(context),
		  m_Context(context),
		  m_Filter()
	{
		LVK_ASSERT(context != nullptr);

        m_Filter.set_timing_samples(TIMING_SAMPLES);
	}

//---------------------------------------------------------------------------------------------------------------------

	void ADBFilter::filter(FrameBuffer& frame)
	{
        LVK_PROFILE;

		if(m_TestMode)
		{
            m_Filter.apply(frame, frame, true);
			draw_debug_hud(frame.data);
		}
		else m_Filter.apply(frame, frame);
	}

//---------------------------------------------------------------------------------------------------------------------

	void ADBFilter::draw_debug_hud(cv::UMat& frame)
	{
        LVK_PROFILE;

		const auto frame_time_ms = m_Filter.timings().average().milliseconds();
		const auto deviation_ms = m_Filter.timings().deviation().milliseconds();

		draw_text(
			frame,
            cv::format("%.2fms (%.2fms)", frame_time_ms, deviation_ms),
			cv::Point(5, 40),
			frame_time_ms < TIMING_THRESHOLD_MS ? yuv::GREEN : yuv::RED
		);
	}

//---------------------------------------------------------------------------------------------------------------------

	bool ADBFilter::validate() const
	{
		return m_Context != nullptr;
	}

//---------------------------------------------------------------------------------------------------------------------

}
