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
#include "Filters/VideoFilter.hpp"
#include "Structures/SlidingBuffer.hpp"
#include "Utility/Properties/Configurable.hpp"

namespace lvk
{

	struct PathStabilizerSettings
	{
		size_t smoothing_frames = 10;
		float correction_margin = 0.1f;
		bool crop_to_margins = false;
		bool lock_focus = true; // TODO: implement (unlocked crop region)
	};

	class PathStabilizer final : public Configurable<PathStabilizerSettings>
	{
	public:

		explicit PathStabilizer(const PathStabilizerSettings& settings = {});
		
		void stabilize(const Frame& input, Frame& output, const Homography& frame_velocity);

		void configure(const PathStabilizerSettings& settings) override;

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
		SlidingBuffer<Frame> m_FrameQueue;
		SlidingBuffer<Homography> m_Trajectory;
		SlidingBuffer<double> m_SmoothingFilter;

		Frame m_NullFrame;
		cv::Rect m_FocusArea{0,0,0,0};
		cv::UMat m_WarpFrame{cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY};
	};

}