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
			const float detection_load,
			const cv::Size& resolution,
			const cv::Size& coarse_block_size,
			const cv::Size& fine_block_size
		);

		void detect(cv::UMat& frame, std::vector<cv::Point2f>& points);

		void propogate(const std::vector<cv::Point2f>& points);

		void reset();

		cv::Size resolution() const;

		size_t detection_target() const;

	private:

		void construct_grids();

		void process_features(std::vector<cv::KeyPoint>& features, const cv::Point2f& offset);

		void extract_features(std::vector<cv::KeyPoint>& features) const;

		bool within_bounds(const cv::Point& point) const;

		size_t coarse_index(const cv::Point& point) const;

		size_t fine_index(const cv::Point& point) const;

	private:

		struct CoarseBlock
		{
			cv::Rect2f bounds;
			int threshold;
			bool active;
		};

		struct FineBlock
		{
			std::optional<cv::KeyPoint> feature;
			bool active;
		};

		const cv::Size m_Resolution;
		const cv::Size m_CoarseBlockSize, m_FineBlockSize;
		const cv::Size m_CoarseGridSize, m_FineGridSize;

		const size_t m_GlobalPointTarget;
		const size_t m_CoarsePointTarget, m_CoarseFeatureTarget;
		const size_t m_FineAreaRatio, m_FineWidthRatio;

		std::vector<cv::Point2f> m_PropogatedPoints;
		std::vector<cv::KeyPoint> m_FeatureBuffer;
		std::vector<CoarseBlock> m_CoarseGrid;
		std::vector<FineBlock> m_FineGrid;
	};

}
