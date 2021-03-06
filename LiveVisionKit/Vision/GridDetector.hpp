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

namespace lvk
{

	class GridDetector
	{
	public:

		GridDetector(
			const cv::Size& resolution,
			const cv::Size& detect_grid_size,
			const cv::Size& feature_grid_size,
			const float detection_load
		);

		void detect(cv::UMat& frame, std::vector<cv::Point2f>& points);

		void propagate(const std::vector<cv::Point2f>& points);

		void reset();

		cv::Size resolution() const;

		size_t feature_capacity() const;

		cv::Point2f distribution_quality() const;

		cv::Point2f distribution_centroid() const;

	private:

		struct DetectBlock
		{
			size_t propagations;
			cv::Rect2f bounds;
			int fast_threshold;
		};

		struct FeatureBlock
		{
			std::optional<cv::KeyPoint> feature;
			bool propagated;
		};

		void construct_grids();

		void process_features(std::vector<cv::KeyPoint>& features, const cv::Point2f& offset);

		void extract_features(std::vector<cv::Point2f>& feature_points) const;

		FeatureBlock& fetch_feature_block(const cv::Point& point);

		DetectBlock& fetch_detect_block(const cv::Point& point);

		bool within_bounds(const cv::Point& point) const;

	private:

		const cv::Size m_Resolution;
		const cv::Size m_DetectGridSize, m_DetectBlockSize;
		const cv::Size m_FeatureGridSize, m_FeatureBlockSize;

		const float m_DetectionLoad;
		const size_t m_FASTFeatureTarget;
		const size_t m_FeaturesPerDetectBlock;
		std::vector<cv::KeyPoint> m_FASTFeatureBuffer;

		std::vector<DetectBlock> m_DetectGrid;
		std::vector<FeatureBlock> m_FeatureGrid;
		std::vector<cv::Point2f> m_FeaturePoints;
	};

}
