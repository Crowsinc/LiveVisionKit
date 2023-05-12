//     *************************** LiveVisionKit ****************************
//     Copyright (C) 2022  Sebastian Di Marco (crowsinc.dev@gmail.com)
//
//     This program is free software: you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation, either version 3 of the License, or
//     (at your option) any later version.
//
//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.
//
//     You should have received a copy of the GNU General Public License
//     along with this program.  If not, see <https://www.gnu.org/licenses/>.
//     **********************************************************************

#include "PathSmoother.hpp"

#include "Functions/Math.hpp"
#include "Logging/CSVLogger.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

    constexpr double MIN_FILTER_SIGMA = 3.0f;
    constexpr double MAX_FILTER_SIGMA = 13.0f;
    constexpr double SIGMA_RESPONSE_RATE = 0.08f;

//---------------------------------------------------------------------------------------------------------------------

	PathSmoother::PathSmoother(const PathSmootherSettings& settings)
        : m_Path(1), // NOTE: initialized properly in configure.
          m_FrameQueue(1) // NOTE: initialized properly in configure.
	{
		configure(settings);
	}

//---------------------------------------------------------------------------------------------------------------------

    void PathSmoother::configure(const PathSmootherSettings& settings)
    {
        LVK_ASSERT(settings.path_prediction_frames > 0);
        LVK_ASSERT_01_STRICT(settings.scene_margins);

        m_Settings = settings;
        configure_buffers();
    }

//---------------------------------------------------------------------------------------------------------------------

    Frame PathSmoother::next(const Frame& frame, const WarpField& motion)
    {
        return next(std::move(frame.clone()), motion);
    }

//---------------------------------------------------------------------------------------------------------------------

    Frame PathSmoother::next(Frame&& frame, const WarpField& motion)
    {
        LVK_ASSERT(!frame.is_empty());

        // Resize past motions if a new size is given.
        if(motion.size() != m_Trace.size())
            resize_fields(motion.size());

        // Update the path's current state
        m_FrameQueue.push(std::move(frame));
        m_Path.advance(motion.size()) = m_Path.newest() + motion;

        if(ready())
        {
            const auto& curr_position = m_Path.centre();
            auto& curr_frame = m_FrameQueue.oldest();

            // Determine the scene margins for the frame size
            m_Margins = crop(curr_frame.size(), m_Settings.scene_margins);
            const cv::Point2f corrective_limits(m_Margins.tl());

            // Determine how much our smoothed path trace has drifted away from the path,
            // as a percentage of the corrective limits (1.0+ => out of scene bounds).
            cv::absdiff(m_Trace.offsets(), curr_position.offsets(), m_Trace.offsets());
            m_Trace /= corrective_limits;

            double min_drift_error, max_drift_error;
            cv::minMaxIdx(m_Trace.offsets(), &min_drift_error, &max_drift_error);
            max_drift_error = std::min(max_drift_error, 1.0);


            // Adapt the smoothing kernel based on the max drift error. If the trace
            // is close to the original path, the smoothing coefficient is raised to
            // maximise the smoothing applied. If the trace starts drifting away from
            // the path and closer to the corrective limits, the smoothing is lowered
            // to bring the trace back towards the path.
            m_SmoothingFactor = exp_moving_average(
                m_SmoothingFactor,
                (MAX_FILTER_SIGMA - MIN_FILTER_SIGMA) * (1.0f - max_drift_error) + MIN_FILTER_SIGMA,
                SIGMA_RESPONSE_RATE
            );

            const auto smoothing_kernel = cv::getGaussianKernel(
                static_cast<int>(m_Path.capacity()), m_SmoothingFactor, CV_32F
            );

            // Apply the filter to get the current smooth trace position.
            m_Trace.set_identity();
            auto weight = smoothing_kernel.ptr<float>();
            for(size_t i = 0; i < m_Path.size(); i++, weight++)
            {
                m_Trace.combine(m_Path[i], *weight);
            }

            // Correct the frame onto the smooth trace position.
            auto path_correction = m_Trace - curr_position;

            // Clamp path to the margins if we are hitting them.
            if(m_Settings.clamp_path_to_margins && max_drift_error == 1.0)
                path_correction.clamp(corrective_limits);

            if(m_Settings.force_output_rigidity)
                path_correction.undistort(m_Settings.rigidity_tolerance);

            if(m_Settings.crop_frame_to_margins)
                path_correction.crop_in(m_Margins, curr_frame.size());

            // NOTE: we perform a swap between the resulting warp frame
            // and the original frame data to ensure zero de-allocations.
            path_correction.apply(curr_frame.data, m_WarpFrame, true);
            std::swap(m_WarpFrame, curr_frame.data);

            return std::move(curr_frame);
        }

        return {};
    }

//---------------------------------------------------------------------------------------------------------------------

    void PathSmoother::restart()
    {
        m_FrameQueue.clear();
        m_Path.clear();

        // Pre-fill the trace to avoid having to deal with edge cases.
        while(!m_Path.is_full()) m_Path.advance(WarpField::MinimumSize);
    }

//---------------------------------------------------------------------------------------------------------------------

    bool PathSmoother::ready() const
    {
        return m_FrameQueue.is_full();
    }

//---------------------------------------------------------------------------------------------------------------------

    size_t PathSmoother::frame_delay() const
    {
        // NOTE: capacity can never be zero, per the pre-conditions
        return m_FrameQueue.capacity() - 1;
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField PathSmoother::position() const
    {
        // NOTE: the path will never be empty.
        return m_Path.centre();
    }

//---------------------------------------------------------------------------------------------------------------------

    const cv::Rect& PathSmoother::stable_region() const
    {
        return m_Margins;
    }

//---------------------------------------------------------------------------------------------------------------------

    void PathSmoother::configure_buffers()
    {
        // The path is held in a circular buffer representing a windowed
        // view on the actual continuous path. The size of the window is
        // based on the number of predictive frames, and is symmetrical
        // with the center position representing the current position in.
        // time. To achieve predictive smoothing, there is a frame queue
        // which delays frames up to match the timing of the path buffer.
        const size_t smoothing_radius = m_Settings.path_prediction_frames;
        const size_t new_window_size = 2 * smoothing_radius + 1;
        const size_t new_queue_size = smoothing_radius + 1;

        if(new_window_size != m_Path.capacity() || new_queue_size != m_FrameQueue.capacity())
        {
            const auto old_queue_size = m_FrameQueue.capacity();

            // When shrinking the buffers, they are both trimmed from the front, so their
            // relative ordering and synchrony is respected. However, resizing the buffers
            // to a larger capacity will move the path forwards as the new center point is
            // pushed to the right, relative to the old path data. So the frames which now
            // corresponded to the path positions that were shifted left towards the past
            // are no longer relevant and need to be skipped.

            m_Path.resize(new_window_size);
            m_FrameQueue.resize(new_queue_size);

            if(new_queue_size > old_queue_size)
            {
                const auto time_shift = new_queue_size - old_queue_size;

                m_FrameQueue.skip(time_shift);
                if(m_FrameQueue.is_empty())
                    restart();
            }
        }
    }

//---------------------------------------------------------------------------------------------------------------------

    void PathSmoother::resize_fields(const cv::Size& new_size)
    {
        m_Trace.resize(new_size);
        for(auto& position : m_Path)
        {
            position.resize(new_size);
        }
    }

//---------------------------------------------------------------------------------------------------------------------

}
