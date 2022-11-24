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

#include <sstream>
#include <util/platform.h>

#include "OBS/Effects/FSREffect.hpp"
#include "OBS/Effects/DefaultEffect.hpp"
#include "OBS/Utility/Locale.hpp"

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

	constexpr auto PROP_MOTION_MODEL = "MOTION_MODEL";
	constexpr auto MOTION_MODEL_AFFINE = "AFFINE";
	constexpr auto MOTION_MODEL_HOMOGRAPHY = "HOMOGRAPHY";
	
	constexpr auto MOTION_MODEL_DYNAMIC = "DYNAMIC";
	constexpr auto MOTION_MODEL_DEFAULT = MOTION_MODEL_DYNAMIC;

	constexpr auto PROP_SUPPRESSION_MODE = "SUPPRESSION_MODE";
	constexpr auto SUPPRESSION_MODE_OFF = "SM_OFF";
	constexpr auto SUPPRESSION_MODE_STRICT = "SM_STRICT";
	constexpr auto SUPPRESSION_MODE_RELAXED = "SM_RELAXED";
	constexpr auto SUPPRESSION_MODE_DEFAULT = SUPPRESSION_MODE_STRICT;

	const auto SUPPRESSION_RANGE_OFF = cv::Point2f(0.0f, 0.0f);
	const auto SUPPRESSION_RANGE_STRICT = cv::Point2f(0.70f, 0.90f);
	const auto SUPPRESSION_RANGE_RELAXED = cv::Point2f(0.0f, 0.30f);
	constexpr auto SUPPRESSION_SMOOTHING_STEP = 3.0f;

	constexpr auto PROP_STAB_DISABLED = "STAB_DISABLED";
	constexpr auto STAB_DISABLED_DEFAULT = false;

	constexpr auto PROP_TEST_MODE = "TEST_MODE";
	constexpr auto TEST_MODE_DEFAULT = false;

	constexpr auto TIMING_THRESHOLD_MS = 6.0;
	constexpr auto MAX_CLAMP_ITERATIONS = 50;

//---------------------------------------------------------------------------------------------------------------------

	obs_properties_t* VSFilter::Properties()
	{
		obs_properties_t* properties = obs_properties_create();

		auto property = obs_properties_add_int(
			properties,
			PROP_SMOOTHING_RADIUS,
			L("vs.radius"),
			SMOOTHING_RADIUS_MIN,
			SMOOTHING_RADIUS_MAX,
			2
		);

		property = obs_properties_add_int(
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

		property = obs_properties_add_list(
			properties,
			PROP_MOTION_MODEL,
			L("vs.model"),
			OBS_COMBO_TYPE_LIST,
			OBS_COMBO_FORMAT_STRING
		);
		obs_property_list_add_string(property, L("vs.model.dynamic"), MOTION_MODEL_DYNAMIC);
		obs_property_list_add_string(property, L("vs.model.affine"), MOTION_MODEL_AFFINE);
		obs_property_list_add_string(property, L("vs.model.homography"), MOTION_MODEL_HOMOGRAPHY);

		property = obs_properties_add_list(
			properties,
			PROP_SUPPRESSION_MODE,
			L("vs.suppression"),
			OBS_COMBO_TYPE_LIST,
			OBS_COMBO_FORMAT_STRING
		);
		obs_property_list_add_string(property, L("vs.suppression.off"), SUPPRESSION_MODE_OFF);
		obs_property_list_add_string(property, L("vs.suppression.strict"), SUPPRESSION_MODE_STRICT);
		obs_property_list_add_string(property, L("vs.suppression.relaxed"), SUPPRESSION_MODE_RELAXED);

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

		obs_data_set_default_int(settings, PROP_SMOOTHING_RADIUS, SMOOTHING_RADIUS_DEFAULT);
		obs_data_set_default_int(settings, PROP_CROP_PERCENTAGE, CROP_PERCENTAGE_DEFAULT);
		obs_data_set_default_string(settings, PROP_MOTION_MODEL, MOTION_MODEL_DEFAULT);
		obs_data_set_default_string(settings, PROP_SUPPRESSION_MODE, SUPPRESSION_MODE_DEFAULT);
		obs_data_set_default_bool(settings, PROP_STAB_DISABLED, STAB_DISABLED_DEFAULT);
		obs_data_set_default_bool(settings, PROP_TEST_MODE, TEST_MODE_DEFAULT);
	}

//---------------------------------------------------------------------------------------------------------------------

	void VSFilter::configure(obs_data_t* settings)
	{
		LVK_ASSERT(settings != nullptr);

		obs_video_info video_info;
		obs_get_video_info(&video_info);
		const float video_fps = static_cast<float>(video_info.fps_num) / video_info.fps_den;
		const float frame_ms = 1000.0/video_fps;

		m_Filter.reconfigure([&](StabilizationSettings& stab_settings) {
			stab_settings.smoothing_frames = round_even(obs_data_get_int(settings, PROP_SMOOTHING_RADIUS));
			stab_settings.crop_proportion = obs_data_get_int(settings, PROP_CROP_PERCENTAGE) / 100.0f;
			stab_settings.stabilize_output = !obs_data_get_bool(settings, PROP_STAB_DISABLED);
			stab_settings.suppression_smoothing_rate = SUPPRESSION_SMOOTHING_STEP / video_fps;

			// Motion Model
			const std::string new_model = obs_data_get_string(settings, PROP_MOTION_MODEL);
			if (new_model == MOTION_MODEL_AFFINE)
				stab_settings.motion_model = MotionModel::AFFINE;
			else if (new_model == MOTION_MODEL_HOMOGRAPHY)
				stab_settings.motion_model = MotionModel::HOMOGRAPHY;
			else if (new_model == MOTION_MODEL_DYNAMIC)
				stab_settings.motion_model = MotionModel::DYNAMIC;

			// Suppression Mode
			const std::string new_mode = obs_data_get_string(settings, PROP_SUPPRESSION_MODE);
			if (new_mode == SUPPRESSION_MODE_STRICT)
			{
				stab_settings.auto_suppression = true;
				stab_settings.suppression_threshold = SUPPRESSION_RANGE_STRICT.y;
				stab_settings.suppression_saturation_limit = SUPPRESSION_RANGE_STRICT.x;
			}
			else if (new_mode == SUPPRESSION_MODE_RELAXED)
			{
				stab_settings.auto_suppression = true;
				stab_settings.suppression_threshold = SUPPRESSION_RANGE_RELAXED.y;
				stab_settings.suppression_saturation_limit = SUPPRESSION_RANGE_RELAXED.x;
			}
			else stab_settings.auto_suppression = false;
		});

		m_TestMode = obs_data_get_bool(settings, PROP_TEST_MODE);

		// Update the frame delay indicator for the user
		const auto old_stream_delay = obs_data_get_int(settings, PROP_STREAM_DELAY_INFO);
		const auto new_stream_delay = static_cast<int>(frame_ms * m_Filter.frame_delay());

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
		  m_Context(context),
		  m_FrameTimer(30)
	{
		LVK_ASSERT(context != nullptr);
	}

//---------------------------------------------------------------------------------------------------------------------

	void VSFilter::hybrid_render(gs_texture_t* frame)
	{
		const auto target = obs_filter_get_target(m_Context);
		const cv::Size render_size(obs_source_get_base_width(target), obs_source_get_base_height(target));
		const cv::Rect render_region = crop(render_size, m_Filter.settings().crop_proportion);

		if (frame == nullptr)
		{
			// As Video Filter
			if (m_TestMode || !FSREffect::Render(m_Context, render_size, render_region))
				obs_source_skip_video_filter(m_Context);
		}
		else
		{
			// As Effects Filter
			if(m_TestMode)
				DefaultEffect::Render(frame);
			else if (!FSREffect::Render(frame, render_size, render_region))
				obs_source_skip_video_filter(m_Context);
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void VSFilter::filter(FrameBuffer& buffer)
	{
		if(is_queue_outdated(buffer))
		{
			m_Filter.restart();
			log::warn("\'%s\' frame queue is outdated, restarting...", obs_source_get_name(m_Context));
		}

		if (m_TestMode)
		{
			m_Filter.profile(buffer, buffer, m_FrameTimer, true);
			if(m_Filter.ready())
				draw_debug_hud(buffer.data);
		}
		else m_Filter.process(buffer, buffer);
	}

//---------------------------------------------------------------------------------------------------------------------

	void VSFilter::draw_debug_hud(cv::UMat& frame)
	{
		const float frame_time_ms = m_FrameTimer.average().milliseconds();
		const float deviation_ms = m_FrameTimer.deviation().milliseconds();
		const auto& crop_region = m_Filter.crop_region();

		draw::text(
			frame,
			std::format("{:.2f}ms ({:.2f}ms)", frame_time_ms, deviation_ms),
			crop_region.tl() + cv::Point(5, 40),
			frame_time_ms < TIMING_THRESHOLD_MS ? draw::YUV_GREEN : draw::YUV_RED
		);
		draw::rect(frame, crop_region, draw::YUV_MAGENTA);
	}

//---------------------------------------------------------------------------------------------------------------------

	bool VSFilter::is_queue_outdated(const FrameBuffer& new_frame)
	{
		// We consider the queue as being outdated if the next frame is over a second 
		// away from the previous frame. To avoid issues with uint64_t underflow, we
		// also consider the queue to be outdated if the previous frame is somehow 
		// newer than the next.

		const auto next_time = new_frame.timestamp;
		const auto curr_time = m_LastTimestamp;
		m_LastTimestamp = new_frame.timestamp;

		return (curr_time > next_time)
			|| (next_time - curr_time > static_cast<uint64_t>(Time::Seconds(1).nanoseconds())); 
	}

//---------------------------------------------------------------------------------------------------------------------

	bool VSFilter::validate() const
	{
		return m_Context != nullptr
			&& FSREffect::IsCompiled();
	}

//---------------------------------------------------------------------------------------------------------------------

}

