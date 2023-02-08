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
#include "GridDetector.hpp"
#include "Math/Homography.hpp"
#include "Utility/Properties/Configurable.hpp"

namespace lvk
{

    struct FrameTrackerSettings
    {
        GridDetector detector = GridDetector();
        size_t minimum_tracking_points = 40;
        cv::Size motion_resolution = {2, 2};
    };

	class FrameTracker final : Configurable<FrameTrackerSettings>
	{
	public:

		explicit FrameTracker(const FrameTrackerSettings& settings = {});

        void configure(const FrameTrackerSettings& settings) override;

		std::optional<WarpField> track(const cv::UMat& next_frame);

		void restart();

		double frame_stability() const;

        double tracking_quality() const;

        const cv::Size& motion_resolution() const;

        const cv::Size& tracking_resolution() const;

		const std::vector<cv::Point2f>& tracking_points() const;

	private:
		std::vector<cv::Point2f> m_TrackedPoints, m_MatchedPoints;
		std::vector<uint8_t> m_MatchStatus, m_InlierStatus;

		double m_FrameStability = 0.0;
        double m_DistributionQuality = 0.0;

		cv::UsacParams m_USACParams;
		cv::Mat m_FilterKernel;

		bool m_FirstFrame = true;
		cv::UMat m_PrevFrame{cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY};
        cv::UMat m_NextFrame{cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY};
	};

}
