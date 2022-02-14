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

#include "../Math/Transform.hpp"

namespace lvk
{

	class FrameTracker
	{
	public:

		FrameTracker(
				const float estimation_threshold = 0.05,
				const cv::Size& resolution = cv::Size(640, 360),
				const cv::Size& block_size = cv::Size(10, 10)
		);

		Transform track(const cv::UMat& next_frame);

		void restart();

	private:

		struct TrackingRegion
		{
			cv::Rect region;
			double feature_threshold;
			uint32_t feature_target;
		};

		const cv::Size m_TrackingResolution;
		const cv::Size m_BlockSize, m_GridSize;
		const uint32_t m_MinMatchThreshold;

		std::vector<cv::KeyPoint> m_Features;
		std::vector<TrackingRegion> m_TrackingRegions;

		std::vector<std::optional<cv::KeyPoint>> m_Grid;
		std::vector<bool> m_GridMask;

		std::vector<cv::Point2f> m_TrackedPoints;
		std::vector<cv::Point2f> m_MatchedPoints;
		std::vector<uint8_t> m_MatchStatus;
		std::vector<float> m_TrackingError;
		std::vector<uint8_t> m_InlierStatus;

		cv::UMat m_PrevFrame, m_NextFrame;
		bool m_FirstFrame;

		cv::Point2f import_next(const cv::UMat& frame);

		void process_features(
			const std::vector<cv::KeyPoint>& features,
			std::vector<cv::Point2f>& points,
			const cv::Point2f& offset
		);

		void update_grid_mask(const std::vector<cv::Point2f>& outliers, const cv::Point2f& scaling);

		void initialise_regions(const uint32_t rows, const uint32_t cols, const uint32_t feature_target);

	};

}
