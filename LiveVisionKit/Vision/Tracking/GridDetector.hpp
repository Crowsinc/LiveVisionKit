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
#include "Structures/SpatialMap.hpp"

namespace lvk
{

	class GridDetector
	{
	public:

		GridDetector(
			const cv::Size& resolution,
			const cv::Size& feature_grid,
			const cv::Size& detection_zones,
            const float detection_threshold,
            const size_t pixels_per_feature = 75
		);

		void detect(cv::UMat& frame, std::vector<cv::Point2f>& points);

		void propagate(const std::vector<cv::Point2f>& points);

		void reset();

		cv::Size resolution() const;

		size_t feature_capacity() const;

        cv::Size local_feature_extent() const;

        cv::Size local_detection_extent() const;

		cv::Point2f distribution_centroid() const;

		double distribution_quality() const;

	private:

		struct DetectZone
		{
            cv::Rect2f bounds;
            int fast_threshold = 0;
            size_t propagations = 0;
		};

		struct FeatureBlock
		{
			cv::KeyPoint feature;
			bool propagated = false;
		};

		void construct_grids();

        void process_features(const std::vector<cv::KeyPoint>& features, const cv::Point2f& offset);

		void extract_features(std::vector<cv::Point2f>& feature_points) const;

	private:

        SpatialMap<FeatureBlock> m_FeatureGrid;
        SpatialMap<DetectZone> m_DetectionZones;

		const size_t m_FASTFeatureTarget, m_MinimumFeatureLoad;
		std::vector<cv::KeyPoint> m_FASTFeatureBuffer;
	};

}
