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

#include "GridDetector.hpp"

#include "Math/Math.hpp"
#include "Diagnostics/Directives.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	constexpr int GLOBAL_FAST_FEATURE_TARGET = 3000;
	constexpr int DEFAULT_FAST_THRESHOLD = 70;
	constexpr int MAX_FAST_THRESHOLD = 250;
	constexpr int MIN_FAST_THRESHOLD = 10;
	constexpr float FAST_THRESHOLD_STEP = 0.1;

//---------------------------------------------------------------------------------------------------------------------
	
	GridDetector::GridDetector(
		const cv::Size& resolution,
		const cv::Size& detect_grid_size,
		const cv::Size& feature_grid_size,
		const float detection_load
	)
		: m_Resolution(resolution),
		  m_DetectGridSize(detect_grid_size),
		  m_DetectBlockSize(
			  resolution.width / detect_grid_size.width,
			  resolution.height / detect_grid_size.height
		  ),
		  m_FeatureGridSize(feature_grid_size),
		  m_FeatureBlockSize(
			  resolution.width / feature_grid_size.width,
			  resolution.height / feature_grid_size.height
		  ),
	      m_DetectionLoad(detection_load),
		  m_FASTFeatureTarget(GLOBAL_FAST_FEATURE_TARGET / m_DetectGridSize.area()),
		  m_FeaturesPerDetectBlock(m_FeatureGridSize.area() / m_DetectGridSize.area())
	{
		LVK_ASSERT(resolution.width > 0);
		LVK_ASSERT(resolution.height > 0);
		LVK_ASSERT(between(detection_load, 0.0f, 1.0f));

		// Grids must evenly divide the resolution
		LVK_ASSERT(resolution.height % detect_grid_size.height == 0);
		LVK_ASSERT(resolution.width % detect_grid_size.width  == 0);
		LVK_ASSERT(resolution.height % feature_grid_size.height == 0);
		LVK_ASSERT(resolution.width % feature_grid_size.width == 0);
		
		// Feature grid must evenly divide the detect grid
		LVK_ASSERT(m_DetectBlockSize.width % m_FeatureBlockSize.width == 0);
		LVK_ASSERT(m_DetectBlockSize.height % m_FeatureBlockSize.height == 0);
		
		// Detect grid must be smaller or equal to feature grid
		// Feature grid must be smaller or equal to resolution
		LVK_ASSERT(between(detect_grid_size.width, 1, feature_grid_size.width));
		LVK_ASSERT(between(detect_grid_size.height, 1, feature_grid_size.height));
		LVK_ASSERT(between(feature_grid_size.width, detect_grid_size.width, resolution.width));
		LVK_ASSERT(between(feature_grid_size.height, detect_grid_size.height, resolution.height));

		m_FASTFeatureBuffer.reserve(m_FASTFeatureTarget);
		m_FeaturePoints.reserve(m_FeatureGridSize.area());

		construct_grids();
		reset();
	}

//---------------------------------------------------------------------------------------------------------------------

	void GridDetector::construct_grids()
	{
		// Construct detection grid
		m_DetectGrid.clear();
		for(int r = 0; r < m_DetectGridSize.height; r++)
		{
			for(int c = 0; c < m_DetectGridSize.width; c++)
			{
				auto& block = m_DetectGrid.emplace_back();
				block.bounds = cv::Rect2f(
					cv::Point2f(c * m_DetectBlockSize.width, r * m_DetectBlockSize.height),
					m_DetectBlockSize
				);
				block.fast_threshold = DEFAULT_FAST_THRESHOLD;
				block.propagations = 0;
			}
		}

		// Construct feature grid
		m_FeatureGrid.clear();
		for(int i = 0; i < m_FeatureGridSize.area(); i++)
		{
			auto& block = m_FeatureGrid.emplace_back();
			block.feature = std::nullopt;
			block.propagated = true;
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void GridDetector::detect(cv::UMat& frame, std::vector<cv::Point2f>& points)
	{
		LVK_ASSERT(frame.size() == resolution());
		LVK_ASSERT(frame.type() == CV_8UC1);

		// Detect new features over the detect grid and process into the feature grid
		for(auto& [propagations, bounds, fast_threshold] : m_DetectGrid)
		{
			const float propagation_load = static_cast<float>(propagations) / m_FeaturesPerDetectBlock;

			if(propagation_load < m_DetectionLoad)
			{
				m_FASTFeatureBuffer.clear();

				cv::FAST(
					frame(bounds),
					m_FASTFeatureBuffer,
					fast_threshold,
					true
				);

				process_features(m_FASTFeatureBuffer, bounds.tl());

				// Dynamically adjust feature threshold to try meet feature target next time
				if(m_FASTFeatureBuffer.size() > m_FASTFeatureTarget)
					fast_threshold = lerp(fast_threshold, MAX_FAST_THRESHOLD, FAST_THRESHOLD_STEP);
				else
					fast_threshold = lerp(fast_threshold, MIN_FAST_THRESHOLD, FAST_THRESHOLD_STEP);
			}
		}

		m_FeaturePoints.clear();
		extract_features(m_FeaturePoints);

		points = m_FeaturePoints;
	}

//---------------------------------------------------------------------------------------------------------------------

	void GridDetector::process_features(std::vector<cv::KeyPoint>& features, const cv::Point2f& offset)
	{
		// Process features into the feature grid, keeping only the best for each block.
		for(cv::KeyPoint feature : features)
		{
			feature.pt.x += offset.x;
			feature.pt.y += offset.y;

			auto& [block_feature, propagated] = fetch_feature_block(feature.pt);
			if(!propagated && (!block_feature.has_value() || block_feature->response < feature.response))
				block_feature = feature;
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void GridDetector::extract_features(std::vector<cv::Point2f>& feature_points) const
	{
		for(const auto& [feature, propagated] : m_FeatureGrid)
			if(feature.has_value())
				feature_points.push_back(feature->pt);
	}

//---------------------------------------------------------------------------------------------------------------------

	void GridDetector::propagate(const std::vector<cv::Point2f>& points)
	{
		reset();

		// Propagate the given points onto the feature grid, 
		for(const auto& point : points)
		{
			// Silently ignore points which are out of bounds
			if (within_bounds(point))
			{
				auto& [feature, active] = fetch_feature_block(point);
				if (!feature.has_value())
				{
					feature = cv::KeyPoint(point, 1);
					active = true;
					
					m_FeaturePoints.push_back(point);
					fetch_detect_block(point).propagations++;
				}
			}
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void GridDetector::reset()
	{
		for (auto& detect_block : m_DetectGrid)
			detect_block.propagations = 0;

		for (auto& [feature, propagated] : m_FeatureGrid)
		{
			feature.reset();
			propagated = false;
		}

		m_FeaturePoints.clear();
	}

//---------------------------------------------------------------------------------------------------------------------

	GridDetector::FeatureBlock& GridDetector::fetch_feature_block(const cv::Point& point)
	{
		LVK_ASSERT(within_bounds(point));

		const size_t block_x = point.x / m_FeatureBlockSize.width;
		const size_t block_y = point.y / m_FeatureBlockSize.height;

		return m_FeatureGrid[block_y * m_FeatureGridSize.width + block_x];
	}

//---------------------------------------------------------------------------------------------------------------------

	GridDetector::DetectBlock& GridDetector::fetch_detect_block(const cv::Point& point)
	{
		LVK_ASSERT(within_bounds(point));

		const size_t block_x = point.x / m_DetectBlockSize.width;
		const size_t block_y = point.y / m_DetectBlockSize.height;

		return m_DetectGrid[block_y * m_DetectGridSize.width + block_x];
	}

//---------------------------------------------------------------------------------------------------------------------

	bool GridDetector::within_bounds(const cv::Point& point) const
	{
		return between(point.x, 0, m_Resolution.width - 1)
			&& between(point.y, 0, m_Resolution.height - 1);
	}

//---------------------------------------------------------------------------------------------------------------------

	cv::Size GridDetector::resolution() const
	{
		return m_Resolution;
	}

//---------------------------------------------------------------------------------------------------------------------

	size_t GridDetector::feature_capacity() const
	{
		return m_FeatureGridSize.area();
	}

//---------------------------------------------------------------------------------------------------------------------

	cv::Point2f GridDetector::distribution_centroid() const
	{
		if(m_FeaturePoints.empty())
			return {0,0};

		cv::Point2f centroid(0, 0);
		for(const auto& point : m_FeaturePoints)
			centroid += point;
		
		centroid /= static_cast<float>(m_FeaturePoints.size());
		
		return centroid;
	}

//---------------------------------------------------------------------------------------------------------------------

	cv::Point2f GridDetector::distribution_quality() const 
	{
		// In terms of the GridDetector, a good distribution of points is 
		// one in which the points evenly and fairly describe the frame.
		// We can define an an ideally distributed set of tracking points 
		// as being perfectly symmetrical across all of the frame's lines
		// of symmetry. Therefore we can measure the distribution quality 
		// by how close the centroid is from the centre of the frame.

		const auto centroid = distribution_centroid();

		// Present independent vertical and horizontal qualities. 
		return cv::Point2f(
			1.0f - 2 * std::abs((centroid.x / m_Resolution.width) - 0.5f),
			1.0f - 2 * std::abs((centroid.y / m_Resolution.height) - 0.5f)
		);
	}

//---------------------------------------------------------------------------------------------------------------------

}


