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

	constexpr int DEFAULT_FAST_THRESHOLD = 70;
	constexpr int MAX_FAST_THRESHOLD = 250;
	constexpr int MIN_FAST_THRESHOLD = 10;
	constexpr float FAST_THRESHOLD_STEP = 0.1;

//---------------------------------------------------------------------------------------------------------------------
	
	GridDetector::GridDetector(
		const cv::Size& resolution,
		const cv::Size& feature_grid,
		const cv::Size& detection_zones,
		const float detection_threshold,
		const size_t pixels_per_feature
	)
		: m_Resolution(resolution),
		  m_DetectGridSize(detection_zones),
		  m_DetectBlockSize(
			  resolution.width / detection_zones.width,
			  resolution.height / detection_zones.height
		  ),
		  m_FeatureGridSize(feature_grid),
		  m_FeatureBlockSize(
			  resolution.width / feature_grid.width,
			  resolution.height / feature_grid.height
		  ),
		  m_FASTFeatureTarget(
			  (resolution.area() / pixels_per_feature) / m_DetectGridSize.area()
		  ),
		  m_MinimumFeatureLoad(static_cast<size_t>(
			  detection_threshold *
			  (static_cast<float>(m_FeatureGridSize.area()) / static_cast<float>(m_DetectGridSize.area()))
		  ))
	{
		LVK_ASSERT(resolution.width > 0);
		LVK_ASSERT(resolution.height > 0);
		LVK_ASSERT(between(detection_threshold, 0.0f, 1.0f));

		// Grids must be smaller or equal to resolution
		LVK_ASSERT(feature_grid.width <= resolution.width);
		LVK_ASSERT(feature_grid.height <= resolution.height);
		LVK_ASSERT(detection_zones.width <= resolution.width);
		LVK_ASSERT(detection_zones.height <= resolution.height);

		// Grids must evenly divide the resolution
		LVK_ASSERT(resolution.height % detection_zones.height == 0);
		LVK_ASSERT(resolution.width % detection_zones.width  == 0);
		LVK_ASSERT(resolution.height % feature_grid.height == 0);
		LVK_ASSERT(resolution.width % feature_grid.width == 0);

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
		for(uint32_t r = 0; r < m_DetectGridSize.height; r++)
		{
			for(uint32_t c = 0; c < m_DetectGridSize.width; c++)
			{
				auto& block = m_DetectGrid.emplace_back();
				block.bounds = cv::Rect2f(
					cv::Point2f(
                        static_cast<float>(c * m_DetectBlockSize.width),
                        static_cast<float>(r * m_DetectBlockSize.height)
                    ),
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
		for(auto& [bounds, fast_threshold, propagations] : m_DetectGrid)
		{
			if(propagations <= m_MinimumFeatureLoad)
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
        calculate_distribution_map();

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
			if(within_bounds(point))
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

    void GridDetector::calculate_distribution_map()
    {
        // Reset distribution map
        for(auto& sector : m_DistributionMap)
            sector = 0.0;

        // Fill the distribution map as a count of each sector's number of points,
        // normalized with respect to the ideal number of points per sector.

        const double normalized_sample_score = static_cast<double>(m_DistributionMap.size())
                                             / static_cast<double>(m_FeaturePoints.size());

        for(const auto& point : m_FeaturePoints)
        {
            const size_t index = spatial_index(point, m_Resolution, m_DistributionMapResolution);
            m_DistributionMap[index] += normalized_sample_score;
        }

        // Calculate global distribution quality by determining the total error from
        // the ideal normalized distribution of 1, then converting to a percentage
        // of the total worst case error. The quality is then the inverse.
        m_GlobalDistributionQuality = 0.0;
        for(const auto& distribution : m_DistributionMap)
            m_GlobalDistributionQuality += std::abs(1.0 - distribution);

        // Given n sectors, the worst case is when all points are in one sector. At which point
        // we have 'n - 1' empty sectors with a distance of 1.0. And one sector with a distance
        // of 'n - 1' as it contains a maximal distribution of 'n'.
        m_GlobalDistributionQuality /= 2.0 * static_cast<double>(m_DistributionMap.size() - 1);
        m_GlobalDistributionQuality = 1.0 - m_GlobalDistributionQuality;
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

	GridDetector::FeatureBlock& GridDetector::fetch_feature_block(const cv::Point2f& point)
	{
		LVK_ASSERT(within_bounds(point));
		return m_FeatureGrid[spatial_index(point, m_Resolution, m_FeatureGridSize)];
	}

//---------------------------------------------------------------------------------------------------------------------

	GridDetector::DetectBlock& GridDetector::fetch_detect_block(const cv::Point2f& point)
	{
		LVK_ASSERT(within_bounds(point));
		return m_DetectGrid[spatial_index(point, m_Resolution, m_DetectGridSize)];
	}

//---------------------------------------------------------------------------------------------------------------------

	bool GridDetector::within_bounds(const cv::Point2f& point) const
	{
		return between<float>(point.x, 0.0f, static_cast<float>(m_Resolution.width) - 1.0f)
			&& between<float>(point.y, 0.0f, static_cast<float>(m_Resolution.height) - 1.0f);
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

	cv::Size GridDetector::local_feature_extent() const
	{
		return m_FeatureBlockSize;
	}

//---------------------------------------------------------------------------------------------------------------------

	cv::Size GridDetector::local_detection_extent() const
	{
		return m_DetectBlockSize;
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

	double GridDetector::distribution_quality() const
	{
		return m_GlobalDistributionQuality;
	}

//---------------------------------------------------------------------------------------------------------------------

    double GridDetector::distribution(const cv::Point2f& location) const
    {
        return m_DistributionMap[spatial_index(location, m_Resolution, m_DistributionMapResolution)];
    }

//---------------------------------------------------------------------------------------------------------------------

}
