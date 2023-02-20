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

	StabilizationFilter::StabilizationFilter(const StabilizationFilterSettings& settings)
		: VideoFilter("Stabilization Filter")
	{
		this->configure(settings);
	}

//---------------------------------------------------------------------------------------------------------------------

    void StabilizationFilter::configure(const StabilizationFilterSettings& settings)
    {
        LVK_ASSERT(between_strict(settings.crop_proportion, 0.0f, 1.0f));
        LVK_ASSERT(between(settings.suppression_threshold, settings.suppression_saturation_limit + 1e-4f, 1.0f));
        LVK_ASSERT(between(settings.suppression_saturation_limit, 0.0f, settings.suppression_threshold - 1e-4f));
        LVK_ASSERT(settings.suppression_smoothing_rate > 0);

        // Reset the tracking when disabling the stabilization otherwise we will have
        // a discontinuity in the tracking once we start up again with a brand new scene.
        if(m_Settings.stabilize_output && !settings.stabilize_output)
            reset_context();

        m_FrameTracker.configure(settings.tracking_settings);
        m_NullMotion.resize(settings.tracking_settings.motion_resolution);

        m_Stabilizer.reconfigure([&](PathStabilizerSettings& path_settings) {
            path_settings.correction_margin = settings.crop_proportion;
            path_settings.smoothing_frames = settings.smoothing_frames;
        });

        m_Settings = settings;
    }

//---------------------------------------------------------------------------------------------------------------------

	void StabilizationFilter::filter(
        Frame&& input,
        Frame& output,
        Stopwatch& timer,
        const bool debug
    )
	{
        LVK_ASSERT(!input.is_empty());

        // Exit early if stabilization is turned off
        if(m_Settings.stabilize_output)
        {
            // Track and stabilize the frame
            cv::extractChannel(input.data, m_TrackingFrame, 0);
            const auto motion = m_FrameTracker.track(m_TrackingFrame);

            if(debug)
            {
                // If we're in debug, draw the motion trackers,
                // ensuring we do not time the debug rendering.
                timer.sync_gpu(debug).pause();
                draw_trackers(input.data);
                timer.sync_gpu(debug).start();
            }

            m_Stabilizer.stabilize(std::move(input), motion.value_or(m_NullMotion), output); // TODO: auto suppression
        }
        else m_Stabilizer.stabilize(std::move(input), m_NullMotion, output);

        if(m_Settings.crop_output && !output.is_empty())
            output.data = output.data(m_Stabilizer.stable_region());
	}

//---------------------------------------------------------------------------------------------------------------------

    void StabilizationFilter::draw_trackers(cv::UMat& frame)
    {
        const cv::Size2f point_scaling(
            static_cast<float>(frame.cols) / static_cast<float>(m_FrameTracker.tracking_resolution().width),
            static_cast<float>(frame.rows) / static_cast<float>(m_FrameTracker.tracking_resolution().height)
        );

        // Draw tracking markers onto frame
        draw::plot_markers(
            frame,
            lerp(draw::YUV_GREEN, draw::YUV_RED, m_SuppressionFactor),
            m_FrameTracker.tracking_points(),
            point_scaling,
            cv::MarkerTypes::MARKER_CROSS,
            8,
            2
        );
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
        reset_context();
	}

//---------------------------------------------------------------------------------------------------------------------

	void StabilizationFilter::reset_context()
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
		return m_FrameTracker.frame_stability();
	}

//---------------------------------------------------------------------------------------------------------------------

	Homography StabilizationFilter::suppress(Homography& motion)
	{
		if (!m_Settings.auto_suppression || !m_Settings.stabilize_output)
		{
			m_SuppressionFactor = 0.0f;
			return motion;
		}
		
		const float scene_stability = m_FrameTracker.frame_stability();
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