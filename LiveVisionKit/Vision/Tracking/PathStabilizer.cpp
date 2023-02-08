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

#include <utility>

#include "Math/Math.hpp"
#include "Math/BoundingQuad.hpp"

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

        m_Settings = settings;
        resize_buffers();
    }

//---------------------------------------------------------------------------------------------------------------------

    std::optional<WarpField> PathStabilizer::stabilize(const Frame& frame, const WarpField& motion, Frame& output)
	{
		LVK_ASSERT(!frame.is_empty());

        // If the given motion has a different resolution, we need
        // to resize the trajectory to match the new field size.
        if(motion.size() != m_Trajectory.newest().size())
            resize_fields(motion.size());

        // Push the incoming frame and associated motion to the queues
		m_FrameQueue.advance().copy(frame);
		m_Trajectory.push(m_Trajectory.newest() + motion);

		if(ready())
		{
			auto& output_frame = m_FrameQueue.oldest();

            // TODO: re-implement with WarpFields!
//            m_Margins = !m_Settings.adaptive_margins ? cv::Rect{0, 0, 0, 0}
//                                                     : crop(output_frame.size(), m_Settings.correction_margin);

            // Calculate the velocity to go from the previous frame's path to the smooth path.
			auto stabilizing_velocity = m_Trajectory.convolve_at(
                m_SmoothingFilter,
                m_Trajectory.centre_index()
            ) - m_Trajectory.centre(-1);

            // TODO: re-implement with WarpFields!
//			if(!m_Margins.empty())
//				stabilizing_velocity = clamp_velocity(stabilizing_velocity, output_frame.size(), m_Margins);

			stabilizing_velocity.warp(output_frame.data, m_WarpFrame);

			// NOTE: we create a new Frame everytime we copy the input to the frame queue
			// so it is safe to directly move the output frame to the output for the user. 
			// Also note that this does mean that the oldest frame will become empty. 
			output_frame.copy(m_Settings.crop_to_margins ? m_WarpFrame(stable_region()) : m_WarpFrame);
			output = std::move(output_frame);

            return std::move(stabilizing_velocity);
		}

        output = std::move(m_NullFrame);
        return std::nullopt;
	}

//---------------------------------------------------------------------------------------------------------------------

    // TODO: re-implement to work with WarpFields!
    Homography PathStabilizer::clamp_velocity(
        const Homography& velocity,
        const cv::Size& frame_size,
        const cv::Rect& focus_area
    )
    {
        // Clamp the velocity so that it respects the correction limits and ensures that
        // the warped frame encloses stable/crop area with no gaps. This is performed by
        // iteratively reducing the velocity by lerping it back to identity in small steps.

        constexpr double max_t = 1.0;
        constexpr double max_iterations = 50;

        constexpr double step = max_t / max_iterations;
        const auto identity = Homography::Identity();

        double t = step;
        Homography reduced_velocity = velocity;
        BoundingQuad frame_bounds(frame_size, reduced_velocity);

        while (t <= max_t && !frame_bounds.encloses(focus_area))
        {
            reduced_velocity = lerp(velocity, identity, t);
            frame_bounds.transform(reduced_velocity);
            t += step;
        }

        return reduced_velocity;
    }

//---------------------------------------------------------------------------------------------------------------------

    bool PathStabilizer::ready() const
    {
        return m_FrameQueue.is_full() && m_Trajectory.is_full();
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
        // NOTE: the trajectory will never be empty.
        return m_Trajectory.newest();
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
		m_Trajectory.clear();

		// Fill the trajectory to bring the buffers into the initial synchronized state.
		while(m_Trajectory.size() < m_FrameQueue.capacity() - 3)
			m_Trajectory.advance(WarpField::MinimumSize);
	}

//---------------------------------------------------------------------------------------------------------------------

	void PathStabilizer::resize_buffers()
	{
		LVK_ASSERT(m_Settings.smoothing_frames >= 2);
		LVK_ASSERT(m_Settings.smoothing_frames % 2 == 0);

		// NOTE: Trajectory uses a full window for stabilising the centre element,
		// so the frame queue needs to supply delayed frames up to the centre.
		// Tracking is performed on the newest frame but the tracked velocity
		// has to be associated with the previous frame, so we add another frame
		// to the queue to introduce an offset.
		const size_t queue_size = m_Settings.smoothing_frames + 2;
		const size_t window_size = 2 * m_Settings.smoothing_frames + 1;

		if(window_size != m_Trajectory.capacity() || queue_size != m_FrameQueue.capacity())
		{
			// NOTE: this is equivalent to the change in smoothing frame count. 
			const auto time_shift = static_cast<int64_t>(queue_size)
                                  - static_cast<int64_t>(m_FrameQueue.capacity());
		
			m_FrameQueue.resize(queue_size);
			m_Trajectory.resize(window_size);

			// When shrinking the buffers, they are both trimmed from the front, 
			// hence their relative ordering and synchrony is respected. However,
			// resizing the buffers to a larger capacity will move the trajectory
			// buffer forwards in time as existing data is pushed to the left of 
			// the new centre point, which represents the current frame in time.
			// The frames correspnding to such data need to be skipped as they 
			// are now in the past. 
			m_FrameQueue.skip(std::max<int64_t>(time_shift, 0));
			if(m_FrameQueue.is_empty())
				reset_buffers();


            // Reset the smoothing filter to match the new window size
            m_SmoothingFilter.resize(window_size);
            m_SmoothingFilter.clear();

            // NOTE: A low pass Gaussian filter is used because it has both decent time domain
            // and frequency domain performance. Unlike an average or windowed sinc filter.
            // As a rule of thumb, sigma is chosen to fit 99.7% of the distribution in the window.
			const auto gaussian_kernel = cv::getGaussianKernel(
                static_cast<int>(window_size),
                static_cast<double>(window_size) / 6.0,
                CV_32F
            );

			for(int i = 0; i < window_size; i++)
				m_SmoothingFilter.push(gaussian_kernel.at<float>(i));
		}
	}

//---------------------------------------------------------------------------------------------------------------------

    void PathStabilizer::resize_fields(const cv::Size& size)
    {
        for(size_t i = 0; i < m_Trajectory.size(); i++)
            m_Trajectory[i].resize(size);
    }

//---------------------------------------------------------------------------------------------------------------------

}