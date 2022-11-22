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

#pragma once

#include <opencv2/opencv.hpp>

#include "Math/Homography.hpp"
#include "Structures/SlidingBuffer.hpp"
#include "Utility/Properties/Configurable.hpp"

namespace lvk
{

	struct FrameVector;
	
	struct PathStabilizerSettings
	{
		bool lock_focus = true; // TODO: implement (unlocked crop region)
		float correction_limit = 1.0f;
		size_t smoothing_frames = 15;
	};

	class PathStabilizer : public Configurable<PathStabilizerSettings>
	{
	public:

		PathStabilizer(const PathStabilizerSettings& settings = {});
		
		const cv::UMat& stabilize(const cv::UMat& frame, const Homography& velocity);

		virtual void configure(const PathStabilizerSettings& settings = {}) override;

		void restart();

		bool ready() const;

		uint32_t frame_delay() const;

		const cv::Rect& stable_region() const;

	private:

		void reset_buffers();

		void resize_buffers();

		static Homography clamp_velocity(
			const Homography& velocity,
			const cv::Size& frame_size,
			const cv::Rect& focus_area
		);

	private:
		SlidingBuffer<cv::UMat> m_FrameQueue;
		SlidingBuffer<FrameVector> m_Trajectory;
		SlidingBuffer<double> m_SmoothingFilter;

		cv::Rect m_FocusArea{0,0,0,0};
		cv::UMat m_NullFrame, m_WarpFrame{cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY};
	};

	struct FrameVector
	{
		Homography displacement;
		Homography velocity;

		FrameVector(
			const Homography& displacement = Homography::Zero(),
			const Homography& velocity = Homography::Zero()
		);

		FrameVector operator+(const Homography& velocity) const;

		FrameVector operator+(const FrameVector& other) const;

		FrameVector operator-(const FrameVector& other) const;

		FrameVector operator*(const double scaling) const;

		FrameVector operator/(const double scaling) const;
	};

}