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

#include "StabilizationFilter.hpp"

#include "Math/Math.hpp"
#include "Utility/Drawing.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	StabilizationFilter::StabilizationFilter(const StabilizationSettings& settings)
		: VideoFilter("Stabilization Filter")
	{
		configure(settings);
	}

//---------------------------------------------------------------------------------------------------------------------

	void StabilizationFilter::process(const Frame& input, Frame& output, const bool debug)
	{
		Homography tracked_motion = Homography::Identity();
		if (m_Settings.stabilize_output)
		{
			cv::extractChannel(input.data, m_TrackingFrame, 0);
			auto tracked_motion = m_FrameTracker.track(m_TrackingFrame);

			if (debug)
			{
				Frame debug_frame = input.clone();
				draw::plot_markers(
					debug_frame.data,
					m_FrameTracker.tracking_points(),
					lerp(draw::YUV_GREEN, draw::YUV_RED, m_SuppressionFactor),
					cv::MarkerTypes::MARKER_CROSS,
					8,
					2
				);

				m_Stabilizer.stabilize(debug_frame, output, tracked_motion);
				return;
			}
		}
		m_Stabilizer.stabilize(input, output, tracked_motion);
	}

//---------------------------------------------------------------------------------------------------------------------

	void StabilizationFilter::configure(const StabilizationSettings& settings)
	{
		LVK_ASSERT(between_strict(settings.crop_proportion, 0.0f, 1.0f));
		LVK_ASSERT(between(settings.suppression_threshold, settings.suppression_saturation_limit + 1e-4f, 1.0f));
		LVK_ASSERT(between(settings.suppression_saturation_limit, 0.0f, settings.suppression_threshold - 1e-4f));
		LVK_ASSERT(settings.suppression_smoothing_rate > 0);

		// Reset the tracking when disabling the stabilization otherwise we will have 
		// a discontinuity in the tracking once we start up again with a brand new scene. 
		if(m_Settings.stabilize_output && !settings.stabilize_output)
			reset_tracking();

		m_Settings = settings;

		m_FrameTracker.set_model(settings.motion_model);
		m_Stabilizer.reconfigure([&](PathStabilizerSettings& path_settings) {
			path_settings.correction_margin = settings.crop_proportion;
		    path_settings.smoothing_frames = settings.smoothing_frames;
			path_settings.crop_to_margins = settings.crop_output;
		});
	}
	
//---------------------------------------------------------------------------------------------------------------------

	bool StabilizationFilter::ready() const
	{
		return m_Stabilizer.ready();
	}

//---------------------------------------------------------------------------------------------------------------------

	void StabilizationFilter::restart()
	{
		m_Stabilizer.restart();
		reset_tracking();
	}

//---------------------------------------------------------------------------------------------------------------------

	void StabilizationFilter::reset_tracking()
	{
		m_FrameTracker.restart();
	}

//---------------------------------------------------------------------------------------------------------------------

	uint32_t StabilizationFilter::frame_delay() const
	{
		return m_Stabilizer.frame_delay();
	}

//---------------------------------------------------------------------------------------------------------------------

	const cv::Rect& StabilizationFilter::crop_region() const
	{
		return m_Stabilizer.stable_region();
	}

//---------------------------------------------------------------------------------------------------------------------

	float StabilizationFilter::stability() const
	{
		return m_FrameTracker.stability();
	}

//---------------------------------------------------------------------------------------------------------------------

	Homography StabilizationFilter::suppress(Homography& motion)
	{
		if (!m_Settings.auto_suppression || !m_Settings.stabilize_output)
		{
			m_SuppressionFactor = 0.0f;
			return motion;
		}
		
		const float scene_stability = m_FrameTracker.stability();
		const float suppression_threshold = m_Settings.suppression_threshold;
		const float saturation_threshold = m_Settings.suppression_saturation_limit;

		float suppression_target = 0.0f;
		if (between(scene_stability, saturation_threshold, suppression_threshold))
		{
			const float length = suppression_threshold - saturation_threshold;
			suppression_target = 1.0f - ((scene_stability - saturation_threshold) / length);
		}
		else if (scene_stability < saturation_threshold)
			suppression_target = 1.0f;

		m_SuppressionFactor = step(
			m_SuppressionFactor,
			suppression_target,
			m_Settings.suppression_smoothing_rate
		);

		return (1.0f - m_SuppressionFactor) * motion + m_SuppressionFactor * Homography::Identity();
	}

//---------------------------------------------------------------------------------------------------------------------
}