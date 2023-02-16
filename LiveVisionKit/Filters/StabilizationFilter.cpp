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

#include <opencv2/core/ocl.hpp>

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
        m_Stabilizer.reconfigure([&](PathStabilizerSettings& path_settings) {
            path_settings.correction_margin = settings.crop_proportion;
            path_settings.smoothing_frames = settings.smoothing_frames;
        });

        m_Settings = settings;
    }

//---------------------------------------------------------------------------------------------------------------------

	void StabilizationFilter::filter(
        const Frame& input,
        Frame& output,
        Stopwatch& timer,
        const bool debug
    )
	{
        LVK_ASSERT(!input.is_empty());

        if(debug) cv::ocl::finish();
        timer.start();

        // Track the frame
        // TODO: make optimized path for when the stabilization is turned off.
        WarpField frame_motion(m_Settings.tracking_settings.motion_resolution);
		if(m_Settings.stabilize_output)
        {
            cv::extractChannel(input.data, m_TrackingFrame, 0);
            if(auto motion = m_FrameTracker.track(m_TrackingFrame); motion.has_value())
                frame_motion = std::move(*motion);
        }

        // Stabilize the input
        if(debug)
        {
            // Ensure we do not time any debug rendering
            cv::ocl::finish();
            timer.pause();

            Frame debug_frame = input.clone();
            if(m_Settings.stabilize_output)
            {
                const auto& tracking_resolution = m_FrameTracker.tracking_resolution();
                const cv::Size2f point_scaling(
                    static_cast<float>(debug_frame.width()) / static_cast<float>(tracking_resolution.width),
                    static_cast<float>(debug_frame.height()) / static_cast<float>(tracking_resolution.height)
                );

                // Draw tracking markers onto frame
                draw::plot_markers(
                    debug_frame.data,
                    lerp(draw::YUV_GREEN, draw::YUV_RED, m_SuppressionFactor),
                    m_FrameTracker.tracking_points(),
                    point_scaling,
                    cv::MarkerTypes::MARKER_CROSS,
                    8,
                    2
                );
            }
            cv::ocl::finish();
            timer.start();

            m_Stabilizer.stabilize(debug_frame, frame_motion, output ); // TODO: suppress(frame_motion));
        }
        else m_Stabilizer.stabilize(input, frame_motion, output); // TODO: suppress(frame_motion));

        if(m_Settings.crop_output && !output.is_empty())
            output.data = output.data(m_Stabilizer.stable_region());

        if(debug) cv::ocl::finish();
        timer.stop();
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