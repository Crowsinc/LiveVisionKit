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

#include "VSFilter.hpp"

#include <util/platform.h>

#include "Effects/FSREffect.hpp"
#include "Utility/ScopedProfiler.hpp"
#include "Utility/Locale.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	constexpr auto PROP_PREDICTIVE_SAMPLES = "SMOOTH_RADIUS";
	constexpr auto PROP_PREDICTIVE_SAMPLES_DEFAULT = 10;
	constexpr auto PROP_PREDICTIVE_SAMPLES_MAX = 20;
	constexpr auto PROP_PREDICTIVE_SAMPLES_MIN = 4;

	constexpr auto PROP_STREAM_DELAY_INFO = "STREAM_DELAY_INFO";
	constexpr auto PROP_STREAM_DELAY_INFO_MAX = 60000;
	constexpr auto PROP_STREAM_DELAY_INFO_MIN = 0;

    constexpr auto PROP_SUBSYSTEM = "MOTION_QUALITY";
    constexpr auto PROP_SUBSYSTEM_HOMOG = "vs.subsystem.1";
    constexpr auto PROP_SUBSYSTEM_FIELD = "vs.subsystem.2";
    constexpr auto PROP_SUBSYSTEM_DEFAULT = PROP_SUBSYSTEM_HOMOG;

	constexpr auto PROP_CROP_PERCENTAGE = "CROP_PERCENTAGE";
	constexpr auto PROP_CROP_PERCENTAGE_DEFAULT = 5;
	constexpr auto PROP_CROP_PERCENTAGE_MAX = 25;
	constexpr auto PROP_CROP_PERCENTAGE_MIN = 1;

    constexpr auto PROP_APPLY_CROP = "APPLY_CROP";
    constexpr auto PROP_APPLY_CROP_DEFAULT = true;

	constexpr auto PROP_STAB_DISABLED = "STAB_DISABLED";
	constexpr auto PROP_STAB_DISABLED_DEFAULT = false;

	constexpr auto PROP_TEST_MODE = "TEST_MODE";
	constexpr auto PROP_TEST_MODE_DEFAULT = false;

	constexpr auto TIMING_THRESHOLD_MS = 6.0;
    constexpr auto TIMING_SAMPLES = 30;

//---------------------------------------------------------------------------------------------------------------------

	obs_properties_t* VSFilter::Properties()
	{
		obs_properties_t* properties = obs_properties_create();

        // Predictive Samples
		obs_properties_add_int(
            properties,
            PROP_PREDICTIVE_SAMPLES,
            L("vs.radius"),
            PROP_PREDICTIVE_SAMPLES_MIN,
            PROP_PREDICTIVE_SAMPLES_MAX,
            1
		);

        // Stream Delay (ms)
		auto property = obs_properties_add_int(
			properties,
			PROP_STREAM_DELAY_INFO,
			L("vs.delay"),
			PROP_STREAM_DELAY_INFO_MIN,
			PROP_STREAM_DELAY_INFO_MAX,
			1
		);
		obs_property_int_set_suffix(property, "ms");
		obs_property_set_enabled(property, false);

        // Motion Quality Selection
        property = obs_properties_add_list(
            properties,
            PROP_SUBSYSTEM,
            L("vs.subsystem"),
            obs_combo_type::OBS_COMBO_TYPE_LIST,
            obs_combo_format::OBS_COMBO_FORMAT_STRING
        );
        obs_property_list_add_string(property, L(PROP_SUBSYSTEM_HOMOG), L(PROP_SUBSYSTEM_HOMOG));
        obs_property_list_add_string(property, L(PROP_SUBSYSTEM_FIELD), L(PROP_SUBSYSTEM_FIELD));

        // Crop Slider
		property = obs_properties_add_int_slider(
			properties,
			PROP_CROP_PERCENTAGE,
			L("f.crop"),
			PROP_CROP_PERCENTAGE_MIN,
			PROP_CROP_PERCENTAGE_MAX,
			1
		);
		obs_property_int_set_suffix(property, "%");

        // Auto-Apply Crop Toggle
        obs_properties_add_bool(
            properties,
            PROP_APPLY_CROP,
            L("vs.apply-crop")
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

        // Disable Stabilization Toggle
		obs_properties_add_bool(
            controls,
			PROP_STAB_DISABLED,
			L("vs.disable")
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

	void VSFilter::LoadDefaults(obs_data_t* settings)
	{
		LVK_ASSERT(settings != nullptr);

		obs_data_set_default_int(settings, PROP_PREDICTIVE_SAMPLES, PROP_PREDICTIVE_SAMPLES_DEFAULT);
		obs_data_set_default_int(settings, PROP_CROP_PERCENTAGE, PROP_CROP_PERCENTAGE_DEFAULT);
		obs_data_set_default_bool(settings, PROP_STAB_DISABLED, PROP_STAB_DISABLED_DEFAULT);
        obs_data_set_default_string(settings, PROP_SUBSYSTEM, PROP_SUBSYSTEM_DEFAULT);
        obs_data_set_default_bool(settings, PROP_APPLY_CROP, PROP_APPLY_CROP_DEFAULT);
		obs_data_set_default_bool(settings, PROP_TEST_MODE, PROP_TEST_MODE_DEFAULT);
	}

//---------------------------------------------------------------------------------------------------------------------

	void VSFilter::configure(obs_data_t* settings)
	{
		LVK_ASSERT(settings != nullptr);

		obs_video_info video_info = {};
		obs_get_video_info(&video_info);
		const float video_fps = static_cast<float>(video_info.fps_num) / static_cast<float>(video_info.fps_den);
		const float frame_ms = 1000.0f/video_fps;

        m_TestMode = obs_data_get_bool(settings, PROP_TEST_MODE);

		m_Filter.reconfigure([&](StabilizationFilterSettings& stab_settings) {
            const auto crop = static_cast<float>(obs_data_get_int(settings, PROP_CROP_PERCENTAGE));

            stab_settings.path_correction_limits = cv::Size2f{crop, crop} / 100.0f;
            stab_settings.crop_to_stable_region = obs_data_get_bool(settings, PROP_APPLY_CROP) && !m_TestMode;
			stab_settings.path_prediction_samples = obs_data_get_int(settings, PROP_PREDICTIVE_SAMPLES);
			stab_settings.stabilize_output = !obs_data_get_bool(settings, PROP_STAB_DISABLED);
            stab_settings.draw_tracking_points = m_TestMode;

            // Configure motion quality
            const std::string subsystem = obs_data_get_string(settings, PROP_SUBSYSTEM);
            if(subsystem == L(PROP_SUBSYSTEM_FIELD))
            {
                stab_settings.detection_resolution = {512, 256};
                stab_settings.motion_resolution = {16, 16};
                stab_settings.detection_regions = {2, 2};

                stab_settings.max_feature_density = 0.15f;
                stab_settings.min_feature_density = 0.03f;
                stab_settings.accumulation_rate = 2.0f;
            }
            else
            {
                stab_settings.detection_resolution = {512, 256};
                stab_settings.motion_resolution = {2, 2};
                stab_settings.detection_regions = {2, 1};

                stab_settings.max_feature_density = 0.10f;
                stab_settings.min_feature_density = 0.02f;
                stab_settings.accumulation_rate = 3.0f;
            }
		});

		// Update the frame delay indicator for the user
		const auto old_stream_delay = obs_data_get_int(settings, PROP_STREAM_DELAY_INFO);
		const auto new_stream_delay = static_cast<int>(frame_ms * static_cast<float>(m_Filter.frame_delay()));

		// NOTE: Need to update the property UI to push a stream delay update because
		// the UI element is disabled. But only if the delay has changed, otherwise
		// the sliders are interrupted and won't smoothly drag anymore.
		if(old_stream_delay != new_stream_delay)
		{
			obs_data_set_int(settings, PROP_STREAM_DELAY_INFO, new_stream_delay);
			obs_source_update_properties(m_Context);
		}


        // Print out settings
        lvk::log::print_settings(
            m_Context,
            "\n    Predictive Frames: %d"
            "\n    Stream Delay: %dms"
            "\n    Subsystem: %s"
            "\n    Crop Percentage: %.0f%%"
            "\n    Auto-apply Crop: %s"
            "\n    Disable Stabilization: %s"
            "\n    Test Mode: %s",
            m_Filter.settings().path_prediction_samples,
            new_stream_delay,
            obs_data_get_string(settings, PROP_SUBSYSTEM),
            m_Filter.settings().path_correction_limits.width * 100.0f,
            m_Filter.settings().crop_to_stable_region ? "Yes" : "No",
            m_Filter.settings().stabilize_output ? "No" : "Yes",
            m_TestMode ? "Yes" : "No"
        );
	}

//---------------------------------------------------------------------------------------------------------------------

	VSFilter::VSFilter(obs_source_t* context)
		: VisionFilter(context),
		  m_Context(context)
	{
		LVK_ASSERT(context != nullptr);

        m_Filter.set_timing_samples(TIMING_SAMPLES);
    }

//---------------------------------------------------------------------------------------------------------------------

	void VSFilter::filter(OBSFrame& frame)
	{
        LVK_PROFILE;

        if(m_TestMode)
        {
            m_Filter.apply(std::move(frame), frame, true);
            draw_debug_hud(frame);
        }
        else m_Filter.apply(std::move(frame), frame);
	}

//---------------------------------------------------------------------------------------------------------------------

	void VSFilter::draw_debug_hud(OBSFrame& frame)
	{
        LVK_PROFILE;

		const double frame_time_ms = m_Filter.timings().average().milliseconds();
		const double deviation_ms = m_Filter.timings().deviation().milliseconds();
		const auto& crop_region = m_Filter.stable_region();

		draw_text(
			frame,
            cv::format("%.2fms (%.2fms)", frame_time_ms, deviation_ms),
			crop_region.tl() + cv::Point(5, 40),
			frame_time_ms < TIMING_THRESHOLD_MS ? col::GREEN[frame.format] : col::RED[frame.format]
		);
		draw_rect(frame, crop_region, col::MAGENTA[frame.format]);
	}

//---------------------------------------------------------------------------------------------------------------------

	bool VSFilter::validate() const
	{
		return m_Context != nullptr
			&& FSREffect::IsCompiled();
	}

//---------------------------------------------------------------------------------------------------------------------

}

