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
#include "Vision/PathStabilizer.hpp"
#include "Utility/Configurable.hpp"

namespace lvk
{

	struct StabilizationFilterSettings
	{
        FrameTrackerSettings tracking_settings;

		size_t smoothing_frames = 10;
		bool stabilize_output = true;

		bool crop_output = false;
		float crop_proportion = 0.05f;
	};


	class StabilizationFilter final : public VideoFilter, public Configurable<StabilizationFilterSettings>
	{
	public:

		explicit StabilizationFilter(const StabilizationFilterSettings& settings = {});

		void configure(const StabilizationFilterSettings& settings) override;

		void restart();

        bool ready() const;

		void reset_context();

		float stability() const;

        size_t frame_delay() const;

		const cv::Rect& crop_region() const;

	private:

        void filter(
            Frame&& input,
            Frame& output,
            Stopwatch& timer,
            const bool debug
        ) override;

	private:
		FrameTracker m_FrameTracker;
		PathStabilizer m_Stabilizer;

        WarpField m_NullMotion{WarpField::MinimumSize};
		cv::UMat m_TrackingFrame{cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY};
	};

}