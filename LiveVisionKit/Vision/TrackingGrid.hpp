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

	class TrackingGrid
	{
	public:

		TrackingGrid(const cv::Size& tracking_resolution, const cv::Size& block_size);

		void process(
			const std::vector<cv::KeyPoint>& keypoints,
			const cv::Point2f& scaling = {1, 1},
			const cv::Point2f& offset = {0, 0}
		);

		void mask(
			const std::vector<cv::Point2f>& points,
			const bool mask_value = false,
			const cv::Point2f& scaling = {1, 1},
			const cv::Point2f& offset = {0, 0}
		);

		void extract(std::vector<cv::Point2f>& points, const int amount = -1);

		void reset_mask(const bool state = true);

		void reset_grid();

		void reset();

		cv::Size grid_size() const;

		cv::Size block_size() const;

		uint32_t block_count() const;

	private:

		const cv::Size m_BlockSize, m_GridSize;

		std::vector<std::optional<cv::KeyPoint>> m_Grid;
		std::vector<bool> m_Mask;
	};





}
