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

#include "PathStabilizer.hpp"
#include "Math/Math.hpp"

#include "Diagnostics/Logging/CSVLogger.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	PathStabilizer::PathStabilizer(const PathStabilizerSettings& settings)
	{
		this->configure(settings);
	}

//---------------------------------------------------------------------------------------------------------------------

    void PathStabilizer::configure(const PathStabilizerSettings& settings)
    {
        LVK_ASSERT(settings.smoothing_frames >= 2);
        LVK_ASSERT(settings.smoothing_frames % 2 == 0);
        LVK_ASSERT(between_strict(settings.correction_margin, 0.0f, 1.0f));
        LVK_ASSERT(between(settings.path_drift_limit, 0.0f, 1.0f));
        LVK_ASSERT(settings.path_drift_rate > 0.0f);

        m_Settings = settings;
        resize_buffers();
    }

//---------------------------------------------------------------------------------------------------------------------

    bool PathStabilizer::ready() const
    {
        return m_FrameQueue.is_full() && m_Trace.is_full();
    }

//---------------------------------------------------------------------------------------------------------------------

    Frame PathStabilizer::next(const Frame& frame, const WarpField& motion)
    {
        return next(std::move(frame.clone()), motion);
    }

//---------------------------------------------------------------------------------------------------------------------

    Frame PathStabilizer::next(Frame&& frame, const WarpField& motion)
	{
        LVK_ASSERT(m_FrameQueue.is_empty() || frame.size() == m_FrameQueue.newest().size());
		LVK_ASSERT(!frame.is_empty());

        m_Margins = crop(frame.size(), m_Settings.correction_margin);
        const cv::Size2f correction_limit(m_Margins.tl());

        if(motion.size() != m_Trace.newest().size())
            rescale_buffers(motion.size());

        WarpField next_position = m_Path.newest() + motion;
        WarpField next_trace = m_Trace.newest();

        // Trace the next position to obtain a constant velocity path
        next_trace.modify([&](cv::Point2f& trace, const cv::Point& position){
            const auto step = next_position.sample(position) - trace;

            if(std::abs(step.x) < m_Settings.path_drift_limit * correction_limit.width)
                trace.x += m_Settings.path_drift_rate * static_cast<float>(sign(step.x));
            else trace.x += step.x;

            if(std::abs(step.y) < m_Settings.path_drift_limit * correction_limit.height)
                trace.y += m_Settings.path_drift_rate * static_cast<float>(sign(step.y));
            else trace.y += step.y;
        });

        // Update all the queues with the new data.
		m_FrameQueue.advance() = std::move(frame);
        m_Path.push(std::move(next_position));
        m_Trace.push(std::move(next_trace));

		if(ready())
		{
            m_SmoothTrace.set_identity();
            for(size_t i = 0; i < m_SmoothingFilter.size(); i++)
            {
                m_SmoothTrace.merge_with(m_Trace[i], m_SmoothingFilter[i]);
            }

            auto path_correction = m_SmoothTrace - m_Path.oldest();
            path_correction.clamp(correction_limit);

            // NOTE: we perform a swap between the resulting warp frame
            // and the original frame data to ensure zero de-allocations.
            auto& next_frame = m_FrameQueue.oldest();
            path_correction.warp(next_frame.data, m_WarpFrame);
            std::swap(m_WarpFrame, next_frame.data);

            return std::move(next_frame);
		}

        return {};
	}

//---------------------------------------------------------------------------------------------------------------------

	void PathStabilizer::restart()
	{
		reset_buffers();
	}

//---------------------------------------------------------------------------------------------------------------------

	size_t PathStabilizer::frame_delay() const
    {
        // NOTE: capacity can never be zero, per the pre-conditions
        return m_FrameQueue.capacity() - 1;
	}

//---------------------------------------------------------------------------------------------------------------------

    WarpField PathStabilizer::raw_position() const
    {
        // NOTE: the path will never be empty.
        return m_Path.newest();
    }

//---------------------------------------------------------------------------------------------------------------------

    WarpField PathStabilizer::stable_position() const
    {
        // NOTE: the trace will never be empty.
        return m_Trace.newest();
    }

//---------------------------------------------------------------------------------------------------------------------

	const cv::Rect& PathStabilizer::stable_region() const
	{
		return m_Margins;
	}

//---------------------------------------------------------------------------------------------------------------------

	void PathStabilizer::reset_buffers()
	{
		m_FrameQueue.clear();
        m_Trace.clear();
		m_Path.clear();

        // Pre-fill the path and trace trajectories to avoid having to deal
        // with synchronization and empty SlidingBuffer edge cases later on.

        while(!m_Trace.is_full()) m_Trace.advance(WarpField::MinimumSize);
        while(!m_Path.is_full()) m_Path.advance(WarpField::MinimumSize);
	}

//---------------------------------------------------------------------------------------------------------------------

	void PathStabilizer::resize_buffers()
	{
		LVK_ASSERT(m_Settings.smoothing_frames >= 2);
		LVK_ASSERT(m_Settings.smoothing_frames % 2 == 0);

        // TODO: re-write this
		// NOTE: The path trace uses a full sized window for stabilising the
        // centre element, so the frame queue needs to supply delayed frames
        // up to the centre. Tracking is performed on the newest frame but
        // the tracked velocity has to be associated with the previous frame,
        // so we add another frame to the queue to introduce an offset. A
		const size_t new_queue_size = m_Settings.smoothing_frames + 2;
		const size_t new_window_size = 2 * m_Settings.smoothing_frames + 1;

		if(new_window_size != m_Trace.capacity() || new_queue_size != m_FrameQueue.capacity())
		{
            const auto old_queue_size = m_FrameQueue.capacity();

            // TODO: re-write this
			// When shrinking the buffers, they are both trimmed from the front, hence their
            // relative ordering and synchrony is respected. However, resizing the buffers to
            // a larger capacity will move the trajectory buffer forwards in time as existing
            // data is pushed to the left of the new centre point, which represents the current
            // frame in time. The frames correspnding to such data need to be skipped as they
            // are now in the past.

            m_Path.resize(new_queue_size);
            m_Trace.resize(new_window_size);
            m_FrameQueue.resize(new_queue_size);

            if(new_queue_size > old_queue_size)
            {
                const auto time_shift = new_queue_size - old_queue_size;

                m_Path.skip(time_shift);
                m_FrameQueue.skip(time_shift);
                if(m_FrameQueue.is_empty() || m_Path.is_empty())
                    reset_buffers();
            }

            // NOTE: A low pass Gaussian filter is used because it has both decent time domain
            // and frequency domain performance. Unlike an average or windowed sinc filter.
            // As a rule of thumb, sigma is chosen to fit 99.7% of the distribution in the window.
            const auto kernel = cv::getGaussianKernel(
                static_cast<int>(new_window_size),
                static_cast<float>(new_window_size) / 6.0f,
                CV_32F
            );

            m_SmoothingFilter.clear();
            m_SmoothingFilter.resize(new_window_size);
            for(int i = 0; i < m_SmoothingFilter.capacity(); i++)
                m_SmoothingFilter.push(kernel.at<float>(i));
		}
	}

//---------------------------------------------------------------------------------------------------------------------

    void PathStabilizer::rescale_buffers(const cv::Size& size)
    {
        m_SmoothTrace.resize(size);

        for(size_t i = 0; i < m_Path.size(); i++)
            m_Path[i].resize(size);

        for(size_t i = 0; i < m_Trace.size(); i++)
            m_Trace[i].resize(size);
    }

//---------------------------------------------------------------------------------------------------------------------

}