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

#include "VideoFilter.hpp"
#include "Vision/FrameTracker.hpp"
#include "Vision/PathSmoother.hpp"
#include "Utility/Configurable.hpp"

namespace lvk
{

	struct StabilizationFilterSettings : public FrameTrackerSettings, public PathSmootherSettings
	{
        float scene_margins = 0.1f;
        bool crop_to_margins = false;
		bool stabilize_output = true;
        bool draw_tracking_points = false;
	};


	class StabilizationFilter final : public VideoFilter, public Configurable<StabilizationFilterSettings>
	{
	public:

		explicit StabilizationFilter(const StabilizationFilterSettings& settings = {});

		void configure(const StabilizationFilterSettings& settings) override;

		void restart();

        bool ready() const;

		void reset_context();

        size_t frame_delay() const;

		float scene_stability() const;

		const cv::Rect& stable_region() const;

        const std::vector<cv::Point2f>& tracking_points() const;

	private:

        void filter(VideoFrame&& input, VideoFrame& output) override;

	private:
		FrameTracker m_FrameTracker;
		PathSmoother m_PathSmoother;

        cv::Rect m_FrameMargins{};
        cv::Size2f m_MotionLimits{};
        StreamBuffer<Frame> m_FrameQueue{1};
        VideoFrame m_WarpFrame, m_TrackingFrame;

        WarpField m_CropMotion{WarpField::MinimumSize};
        WarpField m_NullMotion{WarpField::MinimumSize};
	};

}