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

#include "WarpField.hpp"
#include "Filters/VideoFrame.hpp"
#include "Structures/SlidingBuffer.hpp"
#include "Utility/Properties/Configurable.hpp"

namespace lvk
{

	struct PathStabilizerSettings
	{
        // NOTE: frame delay is proportional to smoothing strength.
		size_t smoothing_strength = 10;
        float scene_margins = 0.1f;

        bool force_rigid_output = true;
        float rigidity_tolerance = 10.0f;
	};

	class PathStabilizer final : public Configurable<PathStabilizerSettings>
	{
	public:

		explicit PathStabilizer(const PathStabilizerSettings& settings = {});

		void configure(const PathStabilizerSettings& settings) override;

		bool ready() const;

        Frame next(const Frame& frame, const WarpField& motion);

        Frame next(Frame&& frame, const WarpField& motion);

		void restart();

		size_t frame_delay() const;

        WarpField position() const;

		const cv::Rect& stable_region() const;

	private:

		void reset_buffers();

		void resize_buffers();

        void rescale_buffers(const cv::Size& size);

	private:
        SlidingBuffer<WarpField> m_Trace;
        SlidingBuffer<float> m_SmoothingFilter;
        WarpField m_SmoothTrace{WarpField::MinimumSize};

        cv::Rect m_Margins{0,0,0,0};
		SlidingBuffer<Frame> m_FrameQueue;
        cv::UMat m_WarpFrame{cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY};
	};

}