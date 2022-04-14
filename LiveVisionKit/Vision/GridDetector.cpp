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

#include "Math/Math.hpp"
#include "Diagnostics/Directives.hpp"
#include "GridDetector.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	constexpr float FEATURE_THRESHOLD_LERP_STEP = 0.1;
	constexpr int DEFAULT_FEATURE_THRESHOLD = 70;
	constexpr int MAX_FEATURE_THRESHOLD = 250;
	constexpr int MIN_FEATURE_THRESHOLD = 10;
	constexpr int GLOBAL_FEATURE_TARGET = 3000;

//---------------------------------------------------------------------------------------------------------------------

	GridDetector::GridDetector(
		const float detection_load,
		const cv::Size& resolution,
		const cv::Size& coarse_block_size,
		const cv::Size& fine_block_size
	)
		: m_Resolution(resolution),
		  m_CoarseBlockSize(coarse_block_size),
		  m_FineBlockSize(fine_block_size),
		  m_CoarseGridSize(
			resolution.width / m_CoarseBlockSize.width,
			resolution.height / m_CoarseBlockSize.height
		  ),
		  m_FineGridSize(
			resolution.width / m_FineBlockSize.width,
			resolution.height / m_FineBlockSize.height
		  ),
		  m_GlobalPointTarget(m_FineGridSize.area() * detection_load),
		  m_CoarsePointTarget(m_GlobalPointTarget / m_CoarseGridSize.area()),
		  m_CoarseFeatureTarget(GLOBAL_FEATURE_TARGET / m_CoarseGridSize.area()),
		  m_FineAreaRatio(m_FineGridSize.area() / m_CoarseGridSize.area()),
		  m_FineWidthRatio(m_FineGridSize.width / m_CoarseGridSize.width)
	{
		LVK_ASSERT(resolution.width > 0);
		LVK_ASSERT(resolution.height > 0);
		LVK_ASSERT(between(detection_load, 0.0f, 1.0f));
		LVK_ASSERT(between(coarse_block_size.width, 1, resolution.width));
		LVK_ASSERT(between(coarse_block_size.height, 1, resolution.height));
		LVK_ASSERT(between(fine_block_size.width, 1, coarse_block_size.width));
		LVK_ASSERT(between(fine_block_size.height, 1, coarse_block_size.height));
		LVK_ASSERT(coarse_block_size.height % fine_block_size.height == 0);
		LVK_ASSERT(coarse_block_size.width % fine_block_size.width == 0);

		m_FeatureBuffer.reserve(m_CoarseFeatureTarget);
		m_PropogatedPoints.reserve(m_FineGridSize.area());

		construct_grids();
		reset();
	}

//---------------------------------------------------------------------------------------------------------------------

	void GridDetector::construct_grids()
	{
		m_CoarseGrid.clear();
		for(int r = 0; r < m_CoarseGridSize.height; r++)
		{
			for(int c = 0; c < m_CoarseGridSize.width; c++)
			{
				auto& block = m_CoarseGrid.emplace_back();
				block.bounds = cv::Rect2f(
					cv::Point2f(c * m_CoarseBlockSize.width, r * m_CoarseBlockSize.height),
					m_CoarseBlockSize
				);
				block.threshold = DEFAULT_FEATURE_THRESHOLD;
				block.active = true;
			}
		}

		m_FineGrid.clear();
		for(int i = 0; i < m_FineGridSize.area(); i++)
		{
			auto& block = m_FineGrid.emplace_back();
			block.feature = std::nullopt;
			block.active = true;
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void GridDetector::reset()
	{
		for(auto& [bounds, threshold, active] : m_CoarseGrid)
			active = true;

		for(auto& [feature, active] : m_FineGrid)
		{
			feature.reset();
			active = true;
		}

		m_PropogatedPoints.clear();
	}

//---------------------------------------------------------------------------------------------------------------------

	void GridDetector::detect(cv::UMat& frame, std::vector<cv::Point2f>& points)
	{
		LVK_ASSERT(frame.size() == resolution());
		LVK_ASSERT(frame.type() == CV_8UC1);

		// Detect new features over the coarse grid and process into the fine grid
		for(auto& [bounds, threshold, active] : m_CoarseGrid)
		{
			if(active)
			{
				m_FeatureBuffer.clear();

				cv::FAST(
					frame(bounds),
					m_FeatureBuffer,
					threshold,
					true
				);

				process_features(m_FeatureBuffer, bounds.tl());

				// Dynamically adjust feature threshold to try meet feature target next time
				if(m_FeatureBuffer.size() > m_CoarseFeatureTarget)
					threshold = lerp(threshold, MAX_FEATURE_THRESHOLD, FEATURE_THRESHOLD_LERP_STEP);
				else
					threshold = lerp(threshold, MIN_FEATURE_THRESHOLD, FEATURE_THRESHOLD_LERP_STEP);
			}
		}

		m_FeatureBuffer.clear();
		extract_features(m_FeatureBuffer);

		for(const auto& feature : m_FeatureBuffer)
			points.push_back(feature.pt);

		for(const auto& point : m_PropogatedPoints)
			points.push_back(point);
	}

//---------------------------------------------------------------------------------------------------------------------

	void GridDetector::propogate(const std::vector<cv::Point2f>& points)
	{
		// Deactivate fine blocks containing propograted points to avoid
		// detecting a new feature in a block which as a propogated feature.
		for(const auto& point : points)
		{
			// Silently ignore points which are out of bounds
			if(within_bounds(point))
			{
				m_PropogatedPoints.push_back(point);
				m_FineGrid[fine_index(point)].active = false;
			}
		}

		// Assuming that an active fine block directly correlates with
		// no propogated points being within that block. We only activate
		// coarse blocks whose active fine blocks come above the point
		// target. In other words, we only detect if we don't already
		// meet the point target through propogated points.
		for(size_t c = 0; c < m_CoarseGrid.size(); c++)
		{
			auto& coarse_block = m_CoarseGrid[c];

			size_t propogated_blocks = 0;
			for(size_t f = 0; coarse_block.active && f < m_FineAreaRatio; f++)
			{
				auto& fine_block = m_FineGrid[c * m_FineAreaRatio + f];

				if(!fine_block.active)
					propogated_blocks++;

				coarse_block.active = propogated_blocks < m_CoarsePointTarget;
			}
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void GridDetector::process_features(std::vector<cv::KeyPoint>& features, const cv::Point2f& offset)
	{
		// Process features into the fine grid, keeping only the best for each block.
		for(auto feature : features)
		{
			feature.pt.x += offset.x;
			feature.pt.y += offset.y;

			LVK_ASSERT(within_bounds(feature.pt));

			auto& [block_feature, active] = m_FineGrid[fine_index(feature.pt)];
			if(active && (!block_feature.has_value() || block_feature->response < feature.response))
				block_feature = feature;
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void GridDetector::extract_features(std::vector<cv::KeyPoint>& features) const
	{
		for(const auto& [feature, active] : m_FineGrid)
			if(active && feature.has_value())
				features.push_back(feature.value());
	}

//---------------------------------------------------------------------------------------------------------------------

	size_t GridDetector::coarse_index(const cv::Point& point) const
	{
		const size_t coarse_x = point.x / m_CoarseBlockSize.width;
		const size_t coarse_y = point.y / m_CoarseBlockSize.height;

		return coarse_y * m_CoarseGridSize.width + coarse_x;
	}

//---------------------------------------------------------------------------------------------------------------------

	size_t GridDetector::fine_index(const cv::Point& point) const
	{
		// NOTE: The fine grid is partitioned such that all fine blocks belonging
		// to a coarse block are contiguous in memory. This allows fast iteration
		// of the fine blocks of a coarse block. Also helps with parallelism.

		const size_t fine_x_offset = (point.x % m_CoarseBlockSize.width) / m_FineBlockSize.width;
		const size_t fine_y_offset = (point.y % m_CoarseBlockSize.height) / m_FineBlockSize.height;

		return coarse_index(point) * m_FineAreaRatio + (fine_y_offset * m_FineWidthRatio + fine_x_offset);
	}

//---------------------------------------------------------------------------------------------------------------------

	bool GridDetector::within_bounds(const cv::Point& point) const
	{
		return between<size_t>(coarse_index(point), 0, m_CoarseGrid.size() - 1);
	}

//---------------------------------------------------------------------------------------------------------------------

	cv::Size GridDetector::resolution() const
	{
		return m_Resolution;
	}

//---------------------------------------------------------------------------------------------------------------------

	size_t GridDetector::detection_target() const
	{
		return m_GlobalPointTarget;
	}

//---------------------------------------------------------------------------------------------------------------------

}


