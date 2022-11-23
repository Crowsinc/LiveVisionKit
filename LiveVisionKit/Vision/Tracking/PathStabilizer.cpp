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
#include "Math/BoundingQuad.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	PathStabilizer::PathStabilizer(const PathStabilizerSettings& settings)
	{
		configure(settings);
	}

//---------------------------------------------------------------------------------------------------------------------

	const cv::UMat& PathStabilizer::stabilize(const cv::UMat& frame, const Homography& velocity)
	{
		LVK_ASSERT(!frame.empty());

		auto& new_frame = m_FrameQueue.advance();
		frame.copyTo(new_frame);

		auto& frame_vector = m_Trajectory.advance();
		frame_vector.velocity = velocity;
		frame_vector.displacement = m_Trajectory.previous().displacement + velocity;

		if(ready())
		{
			auto& next_frame = m_FrameQueue.oldest();
			const auto& [displacement, velocity] = m_Trajectory.centre();

			m_FocusArea = !m_Settings.lock_focus ? cv::Rect{0,0,0,0}
				: crop(next_frame.size(), m_Settings.correction_limit);

			const auto trajectory_correction = m_Trajectory.convolve_at(
				m_SmoothingFilter,
				m_Trajectory.centre_index()
			).displacement - displacement;

			auto stabilizing_velocity = velocity + trajectory_correction;
			if (!m_FocusArea.empty())
				stabilizing_velocity = clamp_velocity(stabilizing_velocity, next_frame.size(), m_FocusArea);

			stabilizing_velocity.warp(next_frame, m_WarpFrame);
			return m_WarpFrame;
		}
		return m_NullFrame;
	}

//---------------------------------------------------------------------------------------------------------------------

	void PathStabilizer::configure(const PathStabilizerSettings& settings)
	{
		LVK_ASSERT(settings.smoothing_frames >= 2);
		LVK_ASSERT(settings.smoothing_frames % 2 == 0);
		LVK_ASSERT(between_strict(settings.correction_limit, 0.0f, 1.0f));
		LVK_ASSERT(settings.lock_focus == true && "THIS IS UNIMPLEMENTED!"); // TODO: implement & remove

		m_Settings = settings;
		resize_buffers();
	}

//---------------------------------------------------------------------------------------------------------------------

	void PathStabilizer::restart()
	{
		reset_buffers();
	}

//---------------------------------------------------------------------------------------------------------------------

	bool PathStabilizer::ready() const
	{
		return m_FrameQueue.full() && m_Trajectory.full();
	}
	
//---------------------------------------------------------------------------------------------------------------------

	uint32_t PathStabilizer::frame_delay() const
	{
		return m_FrameQueue.capacity() - 1;
	}

//---------------------------------------------------------------------------------------------------------------------

	const cv::Rect& PathStabilizer::stable_region() const
	{
		return m_FocusArea;
	}

//---------------------------------------------------------------------------------------------------------------------

	void PathStabilizer::reset_buffers()
	{
		m_FrameQueue.clear();
		m_Trajectory.clear();

		// Fill the trajectory to bring the buffers into the initial synchronized state.
		m_Trajectory.advance(Homography::Identity());
		while(m_Trajectory.size() < m_FrameQueue.capacity() - 3)
			m_Trajectory.advance(m_Trajectory.newest() + Homography::Identity());
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

		if (window_size != m_Trajectory.capacity() || queue_size != m_FrameQueue.capacity())
		{
			// NOTE: this is equivalent to the change in smoothing frame count. 
			const int time_shift = static_cast<int>(queue_size) - m_FrameQueue.capacity();
		
			m_FrameQueue.resize(queue_size);
			m_Trajectory.resize(window_size);

			// When shrinking the buffers, they are both trimmed from the front, 
			// hence their relative ordering and synchrony is respected. However,
			// resizing the buffers to a larger capacity will move the trajectory
			// buffer forwards in time as existing data is pushed to the left of 
			// the new centre point, which represents the current frame in time.
			// The frames correspnding to such data need to be skipped as they 
			// are now in the past. 
			m_FrameQueue.skip(std::max(time_shift, 0));
			if(m_FrameQueue.empty())
				reset_buffers();

			// NOTE: A low pass Gaussian filter is used because it has both decent time domain
			// and frequency domain performance. Unlike an average or windowed sinc filter.
			// As a rule of thumb, sigma is chosen to fit 99.7% of the distribution in the window.
			
			m_SmoothingFilter.resize(window_size);
			m_SmoothingFilter.clear();
		
			const auto gaussian_kernel = cv::getGaussianKernel(window_size, window_size / 6.0);
			for (uint32_t i = 0; i < window_size; i++)
				m_SmoothingFilter.push(gaussian_kernel.at<double>(i));
		}
	}
	
//---------------------------------------------------------------------------------------------------------------------

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

	FrameVector::FrameVector(const Homography& displacement, const Homography& velocity)
		: displacement(displacement),
		  velocity(velocity)
	{}

//---------------------------------------------------------------------------------------------------------------------

	FrameVector FrameVector::operator+(const Homography& velocity) const
	{
		return FrameVector(this->displacement + velocity, this->velocity);
	}

//---------------------------------------------------------------------------------------------------------------------

	FrameVector FrameVector::operator+(const FrameVector& other) const
	{
		return FrameVector(this->displacement + other.displacement, this->velocity + other.velocity);
	}

//---------------------------------------------------------------------------------------------------------------------

	FrameVector FrameVector::operator-(const FrameVector& other) const
	{
		return FrameVector(this->displacement - other.displacement, this->velocity - other.velocity);
	}

//---------------------------------------------------------------------------------------------------------------------

	FrameVector FrameVector::operator*(const double scaling) const
	{
		return FrameVector(this->displacement * scaling, this->velocity * scaling);
	}

//---------------------------------------------------------------------------------------------------------------------

	FrameVector FrameVector::operator/(const double scaling) const
	{
		return FrameVector(this->displacement / scaling, this->velocity / scaling);
	}

//---------------------------------------------------------------------------------------------------------------------

}