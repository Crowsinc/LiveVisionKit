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
#include <optional>

#include "Math/Homography.hpp"
#include "TrackingGrid.hpp"

namespace lvk
{
	enum class MotionModel
	{
		AFFINE,
		HOMOGRAPHY
	};

	class FrameTracker
	{
	public:

		FrameTracker(
			const float estimation_threshold = 0.05,
			const MotionModel model = MotionModel::HOMOGRAPHY,
			const cv::Size& resolution = cv::Size(640, 360),
			const cv::Size& block_size = cv::Size(20, 20)
		);

		Homography track(const cv::UMat& next_frame);

		void restart();

		void set_model(const MotionModel& model);

		MotionModel model() const;

		const std::vector<cv::Point2f> tracking_points() const;

	private:

		struct TrackingRegion
		{
			cv::Rect region;
			double feature_threshold;
			uint32_t detection_target;
		};

		TrackingGrid m_TrackingGrid;
		std::vector<cv::KeyPoint> m_Features;
		std::vector<TrackingRegion> m_TrackingRegions;

		const int m_TrackingPointTarget;
		const uint32_t m_MinMatchThreshold;
		const cv::Size m_TrackingResolution;

		std::vector<cv::Point2f> m_TrackedPoints, m_ScaledTrackedPoints;
		std::vector<cv::Point2f> m_MatchedPoints, m_ScaledMatchedPoints;
		std::vector<uint8_t> m_MatchStatus, m_InlierStatus;
		std::vector<float> m_TrackingError;

		MotionModel m_MotionModel;
		cv::UsacParams m_USACParams;

		cv::UMat m_PrevFrame, m_NextFrame;
		cv::Mat m_FilterKernel;
		bool m_FirstFrame;

		void initialise_regions(const uint32_t rows, const uint32_t cols, const uint32_t detection_target);

		cv::Point2f import_next(const cv::UMat& frame);

		void prepare_state();

	};

}
