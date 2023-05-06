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
#include "Effects/DefaultEffect.hpp"
#include "Utility/Locale.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	constexpr auto PROP_SMOOTHING_RADIUS = "SMOOTH_RADIUS";
	constexpr auto SMOOTHING_RADIUS_DEFAULT = 10;
	constexpr auto SMOOTHING_RADIUS_MIN = 4;
	constexpr auto SMOOTHING_RADIUS_MAX = 20;

	constexpr auto PROP_STREAM_DELAY_INFO = "STREAM_DELAY_INFO";
	constexpr auto STREAM_DELAY_INFO_MIN = 0;
	constexpr auto STREAM_DELAY_INFO_MAX = 100 * SMOOTHING_RADIUS_MAX;

	constexpr auto PROP_CROP_PERCENTAGE = "CROP_PERCENTAGE";
	constexpr auto CROP_PERCENTAGE_DEFAULT = 5;
	constexpr auto CROP_PERCENTAGE_MIN = 1;
	constexpr auto CROP_PERCENTAGE_MAX = 25;

	constexpr auto PROP_SUPPRESSION_THRESH = "SUPPRESSION_THRESH";
    constexpr auto SUPPRESSION_THRESH_DEFAULT = 0.7f;
    constexpr auto SUPPRESSION_THRESH_MIN = 0.0f;
    constexpr auto SUPPRESSION_THRESH_MAX = 1.0f;
    constexpr auto STABILITY_UNIFORMITY_RATIO = 0.5f;

	constexpr auto PROP_STAB_DISABLED = "STAB_DISABLED";
	constexpr auto STAB_DISABLED_DEFAULT = false;

	constexpr auto PROP_TEST_MODE = "TEST_MODE";
	constexpr auto TEST_MODE_DEFAULT = false;

	constexpr auto TIMING_THRESHOLD_MS = 6.0;
    constexpr auto TIMING_SAMPLES = 30;

//---------------------------------------------------------------------------------------------------------------------

	obs_properties_t* VSFilter::Properties()
	{
		obs_properties_t* properties = obs_properties_create();

		obs_properties_add_int(
			properties,
			PROP_SMOOTHING_RADIUS,
			L("vs.radius"),
			SMOOTHING_RADIUS_MIN,
			SMOOTHING_RADIUS_MAX,
			2
		);

		auto property = obs_properties_add_int(
			properties,
			PROP_STREAM_DELAY_INFO,
			L("vs.delay"),
			STREAM_DELAY_INFO_MIN,
			STREAM_DELAY_INFO_MAX,
			1
		);
		obs_property_int_set_suffix(property, "ms");
		obs_property_set_enabled(property, false);

		property = obs_properties_add_int_slider(
			properties,
			PROP_CROP_PERCENTAGE,
			L("f.crop"),
			CROP_PERCENTAGE_MIN,
			CROP_PERCENTAGE_MAX,
			1
		);
		obs_property_int_set_suffix(property, "%");


        obs_properties_add_float_slider(
            properties,
            PROP_SUPPRESSION_THRESH,
            L("vs.suppression"),
            SUPPRESSION_THRESH_MIN,
            SUPPRESSION_THRESH_MAX,
            0.01f
        );

		obs_properties_add_bool(
			properties,
			PROP_STAB_DISABLED,
			L("vs.disable")
		);

		obs_properties_add_bool(
			properties,
			PROP_TEST_MODE,
			L("f.testmode")
		);

		return properties;
	}

//---------------------------------------------------------------------------------------------------------------------

	void VSFilter::LoadDefaults(obs_data_t* settings)
	{
		LVK_ASSERT(settings != nullptr);

        obs_data_set_default_double(settings, PROP_SUPPRESSION_THRESH, SUPPRESSION_THRESH_DEFAULT);
		obs_data_set_default_int(settings, PROP_SMOOTHING_RADIUS, SMOOTHING_RADIUS_DEFAULT);
		obs_data_set_default_int(settings, PROP_CROP_PERCENTAGE, CROP_PERCENTAGE_DEFAULT);
		obs_data_set_default_bool(settings, PROP_STAB_DISABLED, STAB_DISABLED_DEFAULT);
		obs_data_set_default_bool(settings, PROP_TEST_MODE, TEST_MODE_DEFAULT);
	}

//---------------------------------------------------------------------------------------------------------------------

	void VSFilter::configure(obs_data_t* settings)
	{
		LVK_ASSERT(settings != nullptr);

		obs_video_info video_info = {};
		obs_get_video_info(&video_info);
		const float video_fps = static_cast<float>(video_info.fps_num) / static_cast<float>(video_info.fps_den);
		const float frame_ms = 1000.0f/video_fps;

		m_Filter.reconfigure([&](StabilizationFilterSettings& stab_settings) {
			stab_settings.scene_margins = static_cast<float>(obs_data_get_int(settings, PROP_CROP_PERCENTAGE))/100.0f;
			stab_settings.path_prediction_frames = round_even(obs_data_get_int(settings, PROP_SMOOTHING_RADIUS));
			stab_settings.stabilize_output = !obs_data_get_bool(settings, PROP_STAB_DISABLED);

			// Suppression Threshold
            stab_settings.stability_threshold = obs_data_get_double(settings, PROP_SUPPRESSION_THRESH);
            stab_settings.uniformity_threshold = stab_settings.stability_threshold * STABILITY_UNIFORMITY_RATIO;
		});

		m_TestMode = obs_data_get_bool(settings, PROP_TEST_MODE);

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

	void VSFilter::hybrid_render(gs_texture_t* frame)
	{
		const auto target = obs_filter_get_target(m_Context);
		const cv::Size render_size(
            static_cast<int>(obs_source_get_base_width(target)),
            static_cast<int>(obs_source_get_base_height(target))
        );
		const cv::Rect render_region = crop(render_size, m_Filter.settings().scene_margins);

		if(frame == nullptr)
		{
			// As Video Filter
			if(m_TestMode || !FSREffect::Render(m_Context, render_size, render_region))
				obs_source_skip_video_filter(m_Context);
		}
		else
		{
			// As Effects Filter
			if(m_TestMode)
				DefaultEffect::Render(frame);
			else if(!FSREffect::Render(frame, render_size, render_region))
				obs_source_skip_video_filter(m_Context);
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void VSFilter::filter(FrameBuffer& buffer)
	{
		if(m_TestMode)
		{
			m_Filter.process(buffer, buffer, true);
			if(m_Filter.ready())
				draw_debug_hud(buffer.data);
		}
		else m_Filter.process(buffer, buffer);
	}

//---------------------------------------------------------------------------------------------------------------------

	void VSFilter::draw_debug_hud(cv::UMat& frame)
	{
		const double frame_time_ms = m_Filter.timings().average().milliseconds();
		const double deviation_ms = m_Filter.timings().deviation().milliseconds();
		const auto& crop_region = m_Filter.crop_region();

		draw_text(
			frame,
            cv::format("%.2fms (%.2fms)", frame_time_ms, deviation_ms),
			crop_region.tl() + cv::Point(5, 40),
			frame_time_ms < TIMING_THRESHOLD_MS ? yuv::GREEN : yuv::RED
		);
		draw_rect(frame, crop_region, yuv::MAGENTA);
	}

//---------------------------------------------------------------------------------------------------------------------

	bool VSFilter::validate() const
	{
		return m_Context != nullptr
			&& FSREffect::IsCompiled();
	}

//---------------------------------------------------------------------------------------------------------------------

}

