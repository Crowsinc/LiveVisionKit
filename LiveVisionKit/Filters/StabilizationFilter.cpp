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

#include "Directives.hpp"
#include "Functions/Math.hpp"
#include "Functions/Drawing.hpp"
#include "Functions/Extensions.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	StabilizationFilter::StabilizationFilter(const StabilizationFilterSettings& settings)
		: VideoFilter("Stabilization Filter")
	{
		configure(settings);
	}

//---------------------------------------------------------------------------------------------------------------------

    void StabilizationFilter::configure(const StabilizationFilterSettings& settings)
    {
        LVK_ASSERT(settings.smoothing_frames > 0);
        LVK_ASSERT_01_STRICT(settings.crop_proportion);
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
            path_settings.scene_margins = settings.crop_proportion;
            path_settings.path_prediction_frames = settings.smoothing_frames;
            path_settings.motion_resolution = settings.tracking_settings.motion_resolution;
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
        std::optional<WarpField> motion;
        if(m_Settings.stabilize_output)
        {
            // Track and stabilize the frame
            cv::extractChannel(input.data, m_TrackingFrame, 0);
            motion = std::move(m_FrameTracker.track(m_TrackingFrame));

            if(debug)
            {
                // If we're in debug, draw the motion trackers,
                // ensuring we do not time the debug rendering.
                timer.sync_gpu(debug).pause();

                draw_points(
                    input.data,
                    m_FrameTracker.tracking_points(),
                    lerp(yuv::GREEN, yuv::RED, m_SuppressionFactor),
                    3,
                    cv::Size2f(input.size()) / cv::Size2f(m_FrameTracker.tracking_resolution())
                );

                timer.sync_gpu(debug).start();
            }
        }

        output = std::move(m_Stabilizer.next(std::move(input), motion.value_or(m_NullMotion)));

        if(m_Settings.crop_output && !output.is_empty())
            output.data = output.data(m_Stabilizer.stable_region());
	}

//---------------------------------------------------------------------------------------------------------------------

	void StabilizationFilter::restart()
	{
		m_Stabilizer.restart();
        reset_context();
	}

//---------------------------------------------------------------------------------------------------------------------

    bool StabilizationFilter::ready() const
    {
        return m_Stabilizer.ready();
    }

//---------------------------------------------------------------------------------------------------------------------

	void StabilizationFilter::reset_context()
	{
		m_FrameTracker.restart();
	}

//---------------------------------------------------------------------------------------------------------------------

    float StabilizationFilter::stability() const
    {
        return m_FrameTracker.scene_stability();
    }

//---------------------------------------------------------------------------------------------------------------------

    size_t StabilizationFilter::frame_delay() const
    {
        return m_Stabilizer.frame_delay();
    }

//---------------------------------------------------------------------------------------------------------------------

	const cv::Rect& StabilizationFilter::crop_region() const
	{
		return m_Stabilizer.stable_region();
	}

//---------------------------------------------------------------------------------------------------------------------
}