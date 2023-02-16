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
        LVK_ASSERT(settings.adaptive_margins == true && "THIS IS UNIMPLEMENTED!"); // TODO: implement & remove
        LVK_ASSERT(between(settings.path_drift_limit, 0.0f, 1.0f));
        LVK_ASSERT(settings.path_drift_rate > 0.0f);

        m_Settings = settings;
        resize_buffers();
    }

//---------------------------------------------------------------------------------------------------------------------

    std::optional<WarpField> PathStabilizer::stabilize(const Frame& frame, const WarpField& motion, Frame& output)
	{
		LVK_ASSERT(!frame.is_empty() && (m_FrameQueue.is_empty() || frame.size() == m_FrameQueue.newest().size()));

        // If the given motion has a different resolution, we need
        // to resize the trajectory to match the new field size.
        if(motion.size() != m_Path.newest().size())
            resize_fields(motion.size());

        m_Margins = crop(frame.size(), m_Settings.correction_margin);
        const cv::Size2f correction_limit(m_Margins.tl());

        // Push the incoming frame and associated motion to the queues
		m_FrameQueue.advance().copy(frame);
		m_Path.push(m_Path.newest() + motion);

        // TODO: document and rename
        auto step = m_Path.newest() - m_Trace.newest();
        step.modify([&](cv::Point2f& diff, cv::Point position){
            if(std::abs(diff.x) < m_Settings.path_drift_limit * correction_limit.width)
                diff.x = m_Settings.path_drift_rate * static_cast<float>(sign(diff.x));

            if(std::abs(diff.y) < m_Settings.path_drift_limit * correction_limit.height)
                diff.y = m_Settings.path_drift_rate * static_cast<float>(sign(diff.y));
        });
        m_Trace.push(m_Trace.newest() + step);

		if(ready())
		{
			const auto& shaky_frame = m_FrameQueue.oldest();
            const auto& shaky_path = m_Path.oldest();

            auto path_correction = m_Trace.average() - shaky_path;
            path_correction.clamp(correction_limit);

            path_correction.warp(shaky_frame.data, output.data);
            output.timestamp = shaky_frame.timestamp;

            return std::move(path_correction);
		}

        output = std::move(m_NullFrame);
        return std::nullopt;
	}

//---------------------------------------------------------------------------------------------------------------------

    bool PathStabilizer::ready() const
    {
        return m_FrameQueue.is_full() && m_Trace.is_full();
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

    WarpField PathStabilizer::displacement() const
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
		m_Path.clear();
        m_Trace.clear();

		// Fill the trajectory to bring the buffers into the initial synchronized state.
		while(m_Path.size() < m_FrameQueue.capacity() - 3)
        {
			m_Path.advance(WarpField::MinimumSize);
            m_Trace.advance(WarpField::MinimumSize);
        }
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
		}
	}

//---------------------------------------------------------------------------------------------------------------------

    void PathStabilizer::resize_fields(const cv::Size& size)
    {
        for(size_t i = 0; i < m_Path.size(); i++)
        {
            m_Path[i].resize(size);
            m_Trace[i].resize(size);
        }
    }

//---------------------------------------------------------------------------------------------------------------------

}