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

    constexpr auto PROP_QUALITY_ASSURANCE = "SUPPRESSION_MODE";
    constexpr auto PROP_QUALITY_ASSURANCE_STRICT = "SM_STRICT";
    constexpr auto PROP_QUALITY_ASSURANCE_RELAXED = "SM_RELAXED";
    constexpr auto PROP_QUALITY_ASSURANCE_DEFAULT = PROP_QUALITY_ASSURANCE_STRICT;

    constexpr auto PROP_INDEP_CROP = "INDEP_CROP";
    constexpr auto PROP_INDEP_CROP_DEFAULT = false;

    constexpr auto PROP_CROP_PERCENTAGE_X = "CROP_PERCENTAGE_X";
    constexpr auto PROP_CROP_PERCENTAGE_Y = "CROP_PERCENTAGE_Y";
	constexpr auto PROP_CROP_PERCENTAGE_DEFAULT = 5.0f;
	constexpr auto PROP_CROP_PERCENTAGE_MAX = 25.0f;
    constexpr auto PROP_CROP_PERCENTAGE_MIN = 1.0f;
    constexpr auto PROP_CROP_PERCENTAGE_STEP = 0.1f;

    constexpr auto PROP_APPLY_CROP = "APPLY_CROP";
    constexpr auto PROP_APPLY_CROP_DEFAULT = true;

	constexpr auto PROP_BACKGROUND_COLOUR = "BACKGROUND_COL";
	constexpr auto PROP_BACKGROUND_COLOUR_DEFAULT = 0x000000;

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

        // Motion Subsystem Selection
        property = obs_properties_add_list(
            properties,
            PROP_SUBSYSTEM,
            L("vs.subsystem"),
            obs_combo_type::OBS_COMBO_TYPE_LIST,
            obs_combo_format::OBS_COMBO_FORMAT_STRING
        );
        obs_property_list_add_string(property, L(PROP_SUBSYSTEM_HOMOG), L(PROP_SUBSYSTEM_HOMOG));
        obs_property_list_add_string(property, L(PROP_SUBSYSTEM_FIELD), L(PROP_SUBSYSTEM_FIELD));

        property = obs_properties_add_list(
            properties,
            PROP_QUALITY_ASSURANCE,
            L("vs.qa"),
            OBS_COMBO_TYPE_LIST,
            OBS_COMBO_FORMAT_STRING
        );
        obs_property_list_add_string(property, L("vs.qa.relaxed"), PROP_QUALITY_ASSURANCE_RELAXED);
        obs_property_list_add_string(property, L("vs.qa.strict"), PROP_QUALITY_ASSURANCE_STRICT);


        // Independent crop toggle
        property = obs_properties_add_bool(
            properties,
            PROP_INDEP_CROP,
            L("vs.independent-crop")
        );
        obs_property_set_modified_callback(property, VSFilter::on_crop_split);

        // Crop Sliders
		property = obs_properties_add_float_slider(
			properties,
			PROP_CROP_PERCENTAGE_X,
			L("vs.crop-x"),
			PROP_CROP_PERCENTAGE_MIN,
			PROP_CROP_PERCENTAGE_MAX,
            PROP_CROP_PERCENTAGE_STEP
		);
		obs_property_int_set_suffix(property, "%");

        property = obs_properties_add_float_slider(
            properties,
            PROP_CROP_PERCENTAGE_Y,
            L("vs.crop-y"),
            PROP_CROP_PERCENTAGE_MIN,
            PROP_CROP_PERCENTAGE_MAX,
            PROP_CROP_PERCENTAGE_STEP
        );
        obs_property_int_set_suffix(property, "%");

        // Auto-Apply Crop Toggle
        obs_properties_add_bool(
            properties,
            PROP_APPLY_CROP,
            L("vs.apply-crop")
        );

		// Background Colour Wheel
		obs_properties_add_color(
			properties,
			PROP_BACKGROUND_COLOUR,
			L("vs.background-colour")
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

    bool VSFilter::on_crop_split(obs_properties_t* props, obs_property_t* property, obs_data_t* settings)
    {
        auto slider = obs_properties_get(props, PROP_CROP_PERCENTAGE_Y);
        obs_property_set_enabled(slider, obs_data_get_bool(settings, PROP_INDEP_CROP));
        return true;
    }

//---------------------------------------------------------------------------------------------------------------------

	void VSFilter::LoadDefaults(obs_data_t* settings)
	{
		LVK_ASSERT(settings != nullptr);

        obs_data_set_default_string(settings, PROP_QUALITY_ASSURANCE, PROP_QUALITY_ASSURANCE_DEFAULT);
		obs_data_set_default_int(settings, PROP_PREDICTIVE_SAMPLES, PROP_PREDICTIVE_SAMPLES_DEFAULT);
		obs_data_set_default_int(settings, PROP_BACKGROUND_COLOUR, PROP_BACKGROUND_COLOUR_DEFAULT);
        obs_data_set_default_double(settings, PROP_CROP_PERCENTAGE_X, PROP_CROP_PERCENTAGE_DEFAULT);
        obs_data_set_default_double(settings, PROP_CROP_PERCENTAGE_Y, PROP_CROP_PERCENTAGE_DEFAULT);
		obs_data_set_default_bool(settings, PROP_STAB_DISABLED, PROP_STAB_DISABLED_DEFAULT);
        obs_data_set_default_int(settings, PROP_INDEP_CROP, PROP_INDEP_CROP_DEFAULT);
        obs_data_set_default_string(settings, PROP_SUBSYSTEM, PROP_SUBSYSTEM_DEFAULT);
        obs_data_set_default_bool(settings, PROP_APPLY_CROP, PROP_APPLY_CROP_DEFAULT);
		obs_data_set_default_bool(settings, PROP_TEST_MODE, PROP_TEST_MODE_DEFAULT);
	}

//---------------------------------------------------------------------------------------------------------------------

	void VSFilter::configure(obs_data_t* settings)
	{
		LVK_ASSERT(settings != nullptr);

        m_TestMode = obs_data_get_bool(settings, PROP_TEST_MODE);

        // Handle case where independent crop is turned off.
        const bool independent_crop =  obs_data_get_bool(settings, PROP_INDEP_CROP);
        const float crop_x = obs_data_get_double(settings, PROP_CROP_PERCENTAGE_X) * 0.01f;
        const float crop_y = independent_crop ? obs_data_get_double(settings, PROP_CROP_PERCENTAGE_Y) * 0.01f : crop_x;

        m_Filter.reconfigure([&](StabilizationFilterSettings& stab_settings) {
            stab_settings.crop_to_stable_region = obs_data_get_bool(settings, PROP_APPLY_CROP) && !m_TestMode;
			stab_settings.predictive_samples = obs_data_get_int(settings, PROP_PREDICTIVE_SAMPLES);
			stab_settings.stabilize_output = !obs_data_get_bool(settings, PROP_STAB_DISABLED);
            stab_settings.corrective_limits.height = crop_y;
            stab_settings.corrective_limits.width = crop_x;

			// Decode the background colour in RGB.
			uint32_t colour = obs_data_get_int(settings, PROP_BACKGROUND_COLOUR);
			stab_settings.background_colour[0] = static_cast<float>(colour & 0xff);
			stab_settings.background_colour[1] = static_cast<float>((colour >> 8) & 0xff);
			stab_settings.background_colour[2] = static_cast<float>((colour >> 16) & 0xff);

			// Convert the colour to YUV if asynchronous filter.
			if(format() == VideoFrame::YUV || is_asynchronous())
				stab_settings.background_colour = col::rgb2yuv(stab_settings.background_colour);

            // Configure motion subsystem
            const std::string subsystem = obs_data_get_string(settings, PROP_SUBSYSTEM);
            if(subsystem == L(PROP_SUBSYSTEM_FIELD))
            {
                stab_settings.detection_resolution = {480, 270};
                stab_settings.acceptance_threshold = 10.0f;
                stab_settings.track_local_motions = true;
                stab_settings.motion_resolution = {16, 16};
                stab_settings.detection_regions = {2, 2};

                stab_settings.max_feature_density = 0.15f;
                stab_settings.min_feature_density = 0.05f;
                stab_settings.accumulation_rate = 3.0f;
            }
            else
            {
                stab_settings.detection_resolution = {480, 270};
                stab_settings.acceptance_threshold = 3.0f;
                stab_settings.track_local_motions = false;
                stab_settings.motion_resolution = {2, 2};
                stab_settings.detection_regions = {2, 1};

                stab_settings.max_feature_density = 0.08f;
                stab_settings.min_feature_density = 0.02f;
                stab_settings.accumulation_rate = 3.0f;
            }

            // Configure quality assurance
            const std::string quality_assurance = obs_data_get_string(settings, PROP_QUALITY_ASSURANCE);
            if(quality_assurance == PROP_QUALITY_ASSURANCE_STRICT)
                stab_settings.stability_threshold = 0.80f;
            else
                stab_settings.stability_threshold = 0.20f;
		});

        // Get FPS info for the stream.
        obs_video_info video_info = {};
        obs_get_video_info(&video_info);
        const float video_fps = static_cast<float>(video_info.fps_num) / static_cast<float>(video_info.fps_den);

		// Update the frame delay indicator for the user
		const auto old_stream_delay = obs_data_get_int(settings, PROP_STREAM_DELAY_INFO);
		const auto new_stream_delay = static_cast<int>((1000.0f/video_fps) * static_cast<float>(m_Filter.frame_delay()));

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
            "\n    Crop Percentage: (%.0f%%,%.0f%%)"
            "\n    Auto-apply Crop: %s"
            "\n    Disable Stabilization: %s"
            "\n    Test Mode: %s",
            m_Filter.settings().predictive_samples,
            new_stream_delay,
            obs_data_get_string(settings, PROP_SUBSYSTEM),
            m_Filter.settings().corrective_limits.width * 100.0f,
            m_Filter.settings().corrective_limits.height * 100.0f,
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
            m_Filter.draw_motion_mesh();
            m_Filter.draw_trackers();
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

