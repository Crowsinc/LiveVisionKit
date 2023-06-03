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

	StabilizationFilter::StabilizationFilter(const StabilizationFilterSettings& settings)
		: VideoFilter("Stabilization Filter")
	{
		configure(settings);
	}

//---------------------------------------------------------------------------------------------------------------------

    void StabilizationFilter::configure(const StabilizationFilterSettings& settings)
    {
        // Reset the tracking when disabling the stabilization otherwise we will have
        // a discontinuity in the tracking once we start up again with a brand new scene.
        if(m_Settings.stabilize_output && !settings.stabilize_output)
            reset_context();

        // Configure the path smoother and our auxiliary frame queue.
        m_PathSmoother.configure(settings);
        m_FrameQueue.resize(m_PathSmoother.time_delay() + 1);

        m_NullMotion.resize(settings.motion_resolution);
        m_FrameTracker.configure(settings);
        m_Settings = settings;
    }

//---------------------------------------------------------------------------------------------------------------------

	void StabilizationFilter::filter(Frame&& input, Frame& output)
	{
        LVK_ASSERT(!input.is_empty());

        // If we aren't stabilizing the output, use an optimized filter routine that
        // only upkeeps the delay. Note that the path smoothing is reset whenever the
        // output stabilization is turned off, so we do not need to advance the path.
        if(!m_Settings.stabilize_output)
        {
            m_FrameQueue.push(std::move(input));
            if(ready())
            {
                // Swap out the frames to avoid unnecessary allocations.
                std::swap(output, m_FrameQueue.oldest());
                m_FrameQueue.skip(1);
            }
            else output.release();
            return;
        }

        // Track the motion of the incoming frame.
        cv::extractChannel(input.data, m_TrackingFrame, 0);
        const auto motion = m_FrameTracker.track(m_TrackingFrame).value_or(m_NullMotion);

        // Push the tracked frame onto the queue to be stabilized later.
        m_FrameQueue.push(std::move(input));

        m_FrameMargins = crop(m_FrameQueue.oldest().size(), m_Settings.scene_margins);
        const cv::Size2f corrective_limits(m_FrameMargins.tl());

        // If the time delay is properly built up, start stabilizing frames
        if(auto warp = m_PathSmoother.next(motion, corrective_limits); ready())
        {
            // Reference the next frame then skip the buffer by one.
            // This will shorten the queue without de-allocating.
            auto& next_frame = m_FrameQueue.oldest();
            m_FrameQueue.skip();

            // If we need to crop the frame to the scene margins,
            // combine the crop into the path correction warp field.
            if(m_Settings.crop_to_margins)
                warp.crop_in(m_FrameMargins, next_frame.size());

            warp.apply(next_frame.data, output.data);
            output.timestamp = next_frame.timestamp;
        }
        else output.release();
	}

//---------------------------------------------------------------------------------------------------------------------

	void StabilizationFilter::restart()
	{
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

    size_t StabilizationFilter::frame_delay() const
    {
        return m_PathSmoother.time_delay();
    }

//---------------------------------------------------------------------------------------------------------------------

    float StabilizationFilter::scene_stability() const
    {
        return m_FrameTracker.scene_stability();
    }

//---------------------------------------------------------------------------------------------------------------------

	const cv::Rect& StabilizationFilter::stable_region() const
	{
		return m_FrameMargins;
	}

//---------------------------------------------------------------------------------------------------------------------
}