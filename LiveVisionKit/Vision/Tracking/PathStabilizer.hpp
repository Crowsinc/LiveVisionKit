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

#include <optional>
#include <opencv2/opencv.hpp>

#include "WarpField.hpp"
#include "Math/Homography.hpp"
#include "Filters/VideoFrame.hpp"
#include "Structures/SlidingBuffer.hpp"
#include "Utility/Properties/Configurable.hpp"

namespace lvk
{

	struct PathStabilizerSettings
	{
		size_t smoothing_frames = 10;
        float correction_margin = 0.1f;

        float path_drift_rate = 0.25f;
        float path_drift_limit = 0.5f;
	};

	class PathStabilizer final : public Configurable<PathStabilizerSettings>
	{
	public:

		explicit PathStabilizer(const PathStabilizerSettings& settings = {});

		void configure(const PathStabilizerSettings& settings) override;

        std::optional<WarpField> stabilize(Frame&& frame, const WarpField& motion, Frame& output);

        std::optional<WarpField> stabilize(const Frame& frame, const WarpField& motion, Frame& output);

		bool ready() const;

		void restart();

		size_t frame_delay() const;

        WarpField displacement() const;

		const cv::Rect& stable_region() const;

	private:

		void reset_buffers();

		void resize_buffers();

        void resize_fields(const cv::Size& size);

	private:
        cv::Rect m_Margins{0,0,0,0};
        float m_MarginWeight = 1.0f;

        Frame m_NullFrame;
		SlidingBuffer<Frame> m_FrameQueue;
		SlidingBuffer<WarpField> m_Path, m_Trace;
	};

}