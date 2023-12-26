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
#include "Functions/Drawing.hpp"
#include "Functions/Extensions.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

    constexpr float QA_UPDATE_RATE = 0.05f;
    constexpr float QA_BLEND_STEP = 0.05f;

//---------------------------------------------------------------------------------------------------------------------

	StabilizationFilter::StabilizationFilter(const StabilizationFilterSettings& settings)
		: VideoFilter("Stabilization Filter")
	{
		configure(settings);
	}

//---------------------------------------------------------------------------------------------------------------------

    void StabilizationFilter::configure(const StabilizationFilterSettings& settings)
    {
        LVK_ASSERT_01(settings.min_tracking_quality);
        LVK_ASSERT_01(settings.min_scene_quality);

        m_NullMotion.resize(settings.motion_resolution);

        // We need to reset the context when disabling the stabilization
        // otherwise we'll have a discontinuity when start tracking again.
        if(m_Settings.stabilize_output && !settings.stabilize_output)
            reset_context();

        m_Settings = settings;

        // Link up the motion resolutions.
        static_cast<PathSmootherSettings&>(m_Settings).motion_resolution = settings.motion_resolution;
        static_cast<FrameTrackerSettings&>(m_Settings).motion_resolution = settings.motion_resolution;

        // Configure the path smoother and our auxiliary frame queue.
        m_PathSmoother.configure(m_Settings);
        m_FrameQueue.resize(m_PathSmoother.time_delay() + 1);

        m_FrameTracker.configure(m_Settings);
    }

//---------------------------------------------------------------------------------------------------------------------

	void StabilizationFilter::filter(VideoFrame&& input, VideoFrame& output)
	{
        LVK_ASSERT(input.has_known_format());
        LVK_ASSERT(!input.empty());

        // If we aren't stabilizing the output, use an optimized filter routine that
        // only up-keeps the delay. Note that the path smoothing is reset whenever the
        // output stabilization is turned off, so we do not need to advance the path.
        if(!m_Settings.stabilize_output)
        {
            m_FrameQueue.push(std::move(input));
            if(ready())
            {
                // Swap out the frames to avoid unnecessary allocations.
                std::swap(output, m_FrameQueue.oldest());
                m_FrameQueue.skip(1);

                // Apply crop to the output
                if(m_Settings.crop_to_stable_region)
                {
                    m_PathSmoother.scene_crop().apply(output, m_WarpFrame);
                    std::swap(output, m_WarpFrame);
                }
            }
            else output.release();
            return;
        }

        // Track the motion of the incoming frame.
        input.viewAsFormat(m_TrackingFrame, VideoFrame::GRAY);
        auto motion = m_FrameTracker.track(m_TrackingFrame).value_or(m_NullMotion);

        // Apply quality assurance policies
        const auto tracking_quality = m_FrameTracker.tracking_stability();
        m_SceneQuality = exp_moving_average(m_SceneQuality, tracking_quality, QA_UPDATE_RATE);
        if(tracking_quality < m_Settings.min_tracking_quality)
        {
            // This is most likely a discontinuity
            m_TrustFactor = 0.0f;
        }
        else if(m_SceneQuality < m_Settings.min_scene_quality)
            m_TrustFactor = step(m_TrustFactor, 0.0f, QA_BLEND_STEP);
        else
            m_TrustFactor = step(m_TrustFactor, 1.0f, QA_BLEND_STEP);

        // Suppress the motion based on the trust factor
        motion *= m_TrustFactor;

        // Push the tracked frame onto the queue to be stabilized later.
        m_FrameQueue.push(std::move(input));

        // If the time delay is built up, start stabilizing frames
        if(auto correction = m_PathSmoother.next(motion); ready())
        {
            // Reference the next frame then skip the buffer by one.
            // This will shorten the queue without de-allocating.
            auto& next_frame = m_FrameQueue.oldest();
            m_FrameQueue.skip();

            if(m_Settings.crop_to_stable_region)
            {
                correction += m_PathSmoother.scene_crop();
            }
            correction.apply(next_frame, output, m_Settings.background_colour);
        }
        else output.release();
	}

//---------------------------------------------------------------------------------------------------------------------

	void StabilizationFilter::restart()
	{
        m_SceneQuality = 1.0f;
        m_FrameQueue.clear();
        reset_context();
	}

//---------------------------------------------------------------------------------------------------------------------

    bool StabilizationFilter::ready() const
    {
        return m_FrameQueue.is_full();
    }

//---------------------------------------------------------------------------------------------------------------------

	void StabilizationFilter::reset_context()
	{
		m_FrameTracker.restart();
        m_PathSmoother.restart();
	}

//---------------------------------------------------------------------------------------------------------------------

    void StabilizationFilter::draw_trackers()
    {
        auto& frame = m_FrameQueue.newest();
        m_FrameTracker.draw_trackers(
            frame,
            lerp<cv::Scalar,double>(
                col::RED[frame.format],
                col::GREEN[frame.format],
                m_TrustFactor
            ),
            7, 10
        );
    }

//---------------------------------------------------------------------------------------------------------------------

    void StabilizationFilter::draw_motion_mesh()
    {
        auto& frame = m_FrameQueue.newest();
        draw_grid(
            frame,
            m_Settings.motion_resolution - cv::Size{1,1},
            col::BLUE[frame.format],
            1
        );
    }

//---------------------------------------------------------------------------------------------------------------------

    size_t StabilizationFilter::frame_delay() const
    {
        return m_PathSmoother.time_delay();
    }

//---------------------------------------------------------------------------------------------------------------------

	cv::Rect StabilizationFilter::stable_region() const
	{
        const auto& margins = m_PathSmoother.scene_margins();
        const auto& frame_size = cv::Size2f(m_FrameQueue.oldest().size());

        return {margins.tl() * frame_size, margins.size() * frame_size};
	}

//---------------------------------------------------------------------------------------------------------------------

}