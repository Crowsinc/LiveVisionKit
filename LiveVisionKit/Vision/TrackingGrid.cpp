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

#include "TrackingGrid.hpp"

#include "../Math/Math.hpp"
#include "../Diagnostics/Assert.hpp"

#include <iostream>

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	TrackingGrid::TrackingGrid(const cv::Size& tracking_resolution, const cv::Size& block_size)
		: m_BlockSize(block_size),
		  m_GridSize(
			  tracking_resolution.width / block_size.width,
			  tracking_resolution.height/ block_size.height
		  )
	{
		LVK_ASSERT(tracking_resolution.width > 0 && tracking_resolution.height > 0);
		LVK_ASSERT(between(block_size.width, 1, tracking_resolution.width + 1));
		LVK_ASSERT(between(block_size.height, 1, tracking_resolution.height + 1));

		m_Grid.resize(m_GridSize.area());
		m_Mask.resize(m_GridSize.area(), true);
	}

//---------------------------------------------------------------------------------------------------------------------

	void TrackingGrid::process(
		const std::vector<cv::KeyPoint>& keypoints,
		const cv::Point2f& scaling,
		const cv::Point2f& offset
	)
	{
		// Process keypoints into the grid, keeping only the best for each block.
		for(const auto& kp : keypoints)
		{
			const cv::Point2f point(
				kp.pt.x * scaling.x + offset.x,
				kp.pt.y * scaling.y + offset.y
			);

			const uint32_t grid_x = point.x / m_BlockSize.width;
			const uint32_t grid_y = point.y / m_BlockSize.height;
			const size_t index = grid_y * m_GridSize.width + grid_x;

			// Silently skip points that land outside the grid
			if(!between<size_t>(index, 0, m_Grid.size()))
				continue;

			auto& block = m_Grid[index];
			const bool block_mask = m_Mask[index];
			if(block_mask == true && (!block.has_value() || block->response < kp.response))
			{
				block = kp;
				block->pt = point;
			}
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void TrackingGrid::mask(
		const std::vector<cv::Point2f>& points,
		const bool mask_value,
		const cv::Point2f& scaling,
		const cv::Point2f& offset
	)
	{
		// Update the mask state of the provided blocks
		for(auto point : points)
		{
			point.x = point.x * scaling.x + offset.x;
			point.y = point.y * scaling.y + offset.y;

			const uint32_t grid_x = point.x / m_BlockSize.width;
			const uint32_t grid_y = point.y / m_BlockSize.height;
			const size_t index = grid_y * m_GridSize.width + grid_x;

			// Silently skip points that land outside the grid
			if(!between<size_t>(index, 0, m_Grid.size()))
				continue;

			m_Mask[index] = mask_value;
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void TrackingGrid::extract(std::vector<cv::Point2f>& points, const int amount)
	{
		std::vector<cv::KeyPoint> kps;
		for(const auto& kp : m_Grid)
			if(kp.has_value())
				kps.push_back(kp.value());

		if(amount >= 0)
			cv::KeyPointsFilter::retainBest(kps, amount);

		for(const auto& kp : kps)
			points.push_back(kp.pt);
	}

//---------------------------------------------------------------------------------------------------------------------

	void TrackingGrid::reset_mask(const bool state)
	{
		for(uint32_t i = 0; i < m_Mask.size(); i++)
			m_Mask[i] = state;
	}

//---------------------------------------------------------------------------------------------------------------------

	void TrackingGrid::reset_grid()
	{
		for(auto& kp : m_Grid)
			kp.reset();
	}

//---------------------------------------------------------------------------------------------------------------------

	void TrackingGrid::reset()
	{
		reset_grid();
		reset_mask();
	}

//---------------------------------------------------------------------------------------------------------------------

	cv::Size TrackingGrid::grid_size() const
	{
		return m_GridSize;
	}

//---------------------------------------------------------------------------------------------------------------------

	cv::Size TrackingGrid::block_size() const
	{
		return m_BlockSize;
	}

//---------------------------------------------------------------------------------------------------------------------

	uint32_t TrackingGrid::block_count() const
	{
		return m_GridSize.area();
	}

//---------------------------------------------------------------------------------------------------------------------

}
