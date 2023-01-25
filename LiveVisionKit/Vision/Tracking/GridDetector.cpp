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
		: m_FeatureGrid(feature_grid, cv::Rect(cv::Point(0,0), resolution)),
          m_DetectionZones(detection_zones, cv::Rect(cv::Point(0,0), resolution)),
		  m_FASTFeatureTarget(
			  (resolution.area() / pixels_per_feature) / detection_zones.area()
		  ),
		  m_MinimumFeatureLoad(static_cast<size_t>(
			  detection_threshold *
			  (static_cast<float>(feature_grid.area()) / static_cast<float>(detection_zones.area()))
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
		construct_grids();
		reset();
	}

//---------------------------------------------------------------------------------------------------------------------

	void GridDetector::construct_grids()
	{
		// Construct detection zones
        m_DetectionZones.clear();
		for(size_t r = 0; r < m_DetectionZones.rows(); r++)
		{
			for(size_t c = 0; c < m_DetectionZones.cols(); c++)
			{
                DetectZone zone;
                zone.bounds = cv::Rect2f(
					cv::Point2f(
                        static_cast<float>(c) * m_DetectionZones.key_size().width,
                        static_cast<float>(r) * m_DetectionZones.key_size().height
                    ),
					m_DetectionZones.key_size()
				);
				zone.fast_threshold = DEFAULT_FAST_THRESHOLD;
				zone.propagations = 0;

                m_DetectionZones.place_at({c, r}, zone);
			}
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void GridDetector::detect(cv::UMat& frame, std::vector<cv::Point2f>& points)
	{
		LVK_ASSERT(frame.size() == resolution());
		LVK_ASSERT(frame.type() == CV_8UC1);

		// Detect new features over the detect grid and process into the feature grid
		for(auto& [coord, zone] : m_DetectionZones)
		{
            auto& [bounds, fast_threshold, propagations] = zone;

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

		extract_features(points);
	}

//---------------------------------------------------------------------------------------------------------------------

	void GridDetector::process_features(const std::vector<cv::KeyPoint>& features, const cv::Point2f& offset)
	{
		// Process features into the feature grid, keeping only the best for each block.
		for(cv::KeyPoint feature : features)
		{
			feature.pt.x += offset.x;
			feature.pt.y += offset.y;

			auto& [block_feature, propagated] = m_FeatureGrid[feature.pt];
			if(!propagated && block_feature.response < feature.response)
				block_feature = feature;
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void GridDetector::extract_features(std::vector<cv::Point2f>& feature_points) const
	{
		for(const auto& [coord, block] : m_FeatureGrid)
            feature_points.push_back(block.feature.pt);
    }

//---------------------------------------------------------------------------------------------------------------------

	void GridDetector::propagate(const std::vector<cv::Point2f>& points)
	{
		reset();

		// Propagate the given points onto the feature grid,
		for(const auto& point : points)
		{
			// Silently ignore points which are out of bounds
			if(m_FeatureGrid.within_bounds(point))
			{
                // Ignore point if one has already been propagated to the same point.
                if(m_FeatureGrid.contains(m_FeatureGrid.key_of(point)))
                    continue;

				auto& [feature, active] = m_FeatureGrid[point];
                feature = cv::KeyPoint(point, 1);
                active = true;
					
                m_DetectionZones[point].propagations++;
			}
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void GridDetector::reset()
	{
		for (auto& [coord, zone] : m_DetectionZones)
			zone.propagations = 0;

        m_FeatureGrid.clear();
	}

//---------------------------------------------------------------------------------------------------------------------

	cv::Size GridDetector::resolution() const
	{
		return m_FeatureGrid.alignment().size();
	}

//---------------------------------------------------------------------------------------------------------------------

	size_t GridDetector::feature_capacity() const
	{
		return m_FeatureGrid.capacity();
	}

//---------------------------------------------------------------------------------------------------------------------

	cv::Size GridDetector::local_feature_extent() const
	{
		return cv::Size(m_FeatureGrid.key_size());
	}

//---------------------------------------------------------------------------------------------------------------------

	cv::Size GridDetector::local_detection_extent() const
	{
		return cv::Size(m_DetectionZones.key_size());
	}

//---------------------------------------------------------------------------------------------------------------------

	cv::Point2f GridDetector::distribution_centroid() const
	{
		return m_FeatureGrid.distribution_centroid<float>();
	}

//---------------------------------------------------------------------------------------------------------------------

	double GridDetector::distribution_quality() const
	{
		return m_FeatureGrid.distribution_quality();
	}

//---------------------------------------------------------------------------------------------------------------------

}
