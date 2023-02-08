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
	
	GridDetector::GridDetector(const GridDetectorSettings& settings)
        : m_FeatureGrid(settings.feature_grid_shape),
          m_DetectionZones(settings.detection_zones),
          m_MinimumFeatureLoad(0),  // Set by configure()
          m_FASTFeatureTarget(0)    // Set by configure()
	{
        this->configure(settings);
        construct_detection_zones();
	}

//---------------------------------------------------------------------------------------------------------------------

    void GridDetector::configure(const GridDetectorSettings& settings)
    {
        LVK_ASSERT(settings.input_resolution.width > 0);
        LVK_ASSERT(settings.input_resolution.height > 0);
        LVK_ASSERT(between(settings.detection_threshold, 0.0f, 1.0f));

        // Grids must be smaller or equal to resolution
        LVK_ASSERT(settings.feature_grid_shape.width <= settings.input_resolution.width);
        LVK_ASSERT(settings.feature_grid_shape.height <= settings.input_resolution.height);
        LVK_ASSERT(settings.detection_zones.width <= settings.input_resolution.width);
        LVK_ASSERT(settings.detection_zones.height <= settings.input_resolution.height);

        // Grids must evenly divide the resolution
        LVK_ASSERT(settings.input_resolution.height % settings.detection_zones.height == 0);
        LVK_ASSERT(settings.input_resolution.width % settings.detection_zones.width == 0);
        LVK_ASSERT(settings.input_resolution.height % settings.feature_grid_shape.height == 0);
        LVK_ASSERT(settings.input_resolution.width % settings.feature_grid_shape.width == 0);

        if(m_Settings.feature_grid_shape != settings.feature_grid_shape)
        {
            reset();
            m_FeatureGrid.rescale(settings.feature_grid_shape);
        }

        if(m_Settings.detection_zones != settings.detection_zones)
        {
            m_DetectionZones.clear();
            m_DetectionZones.rescale(settings.detection_zones);
            construct_detection_zones();
        }

        const cv::Rect input_region({0,0}, settings.input_resolution);
        m_DetectionZones.align(input_region);
        m_FeatureGrid.align(input_region);

        const auto input_area = static_cast<float>(settings.input_resolution.area());
        const auto detect_zones = static_cast<float>(settings.detection_zones.area());
        const auto feature_blocks = static_cast<float>(settings.feature_grid_shape.area());

        m_MinimumFeatureLoad = static_cast<size_t>((settings.detection_threshold * feature_blocks) / detect_zones);
        m_FASTFeatureTarget = static_cast<size_t>((input_area * settings.detection_density) / detect_zones);
        m_FASTFeatureBuffer.reserve(m_FASTFeatureTarget);

        m_Settings = settings;
    }

//---------------------------------------------------------------------------------------------------------------------

	void GridDetector::construct_detection_zones()
	{
        const cv::Size2f detection_zone_size = m_DetectionZones.key_size();

        m_DetectionZones.clear();
		for(size_t r = 0; r < m_DetectionZones.rows(); r++)
		{
			for(size_t c = 0; c < m_DetectionZones.cols(); c++)
			{
                DetectZone zone;
                zone.bounds = cv::Rect2f(
                    static_cast<float>(c) * detection_zone_size.width,
                    static_cast<float>(r) * detection_zone_size.height,
                    detection_zone_size.width,
                    detection_zone_size.height
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
		LVK_ASSERT(frame.size() == input_resolution());
		LVK_ASSERT(frame.type() == CV_8UC1);

		// Detect new features over the detect grid and process into the feature grid
		for(auto& [coord, detection_zone] : m_DetectionZones)
		{
            auto& [bounds, fast_threshold, propagations] = detection_zone;

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
            LVK_ASSERT(m_FeatureGrid.within_bounds(feature.pt));

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
			if(const auto& key = m_FeatureGrid.try_key_of(point); key.has_value())
			{
                // Ignore point if one has already been propagated to the same point.
                if(m_FeatureGrid.contains(*key))
                    continue;

				m_FeatureGrid.emplace_at(*key, cv::KeyPoint(point, 1), true);
                m_DetectionZones[point].propagations++;
			}
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void GridDetector::reset()
	{
		for(auto& [coord, zone] : m_DetectionZones)
			zone.propagations = 0;

        m_FeatureGrid.clear();
	}

//---------------------------------------------------------------------------------------------------------------------

    size_t GridDetector::feature_capacity() const
    {
        return m_FeatureGrid.capacity();
    }

//---------------------------------------------------------------------------------------------------------------------

	cv::Size GridDetector::local_feature_size() const
	{
		return cv::Size(m_FeatureGrid.key_size());
	}

//---------------------------------------------------------------------------------------------------------------------

	cv::Size GridDetector::detection_zone_size() const
	{
		return cv::Size(m_DetectionZones.key_size());
	}

//---------------------------------------------------------------------------------------------------------------------

    double GridDetector::distribution_quality() const
    {
        return m_FeatureGrid.distribution_quality();
    }

//---------------------------------------------------------------------------------------------------------------------

	cv::Point2f GridDetector::distribution_centroid() const
	{
		return m_FeatureGrid.distribution_centroid<float>();
	}

//---------------------------------------------------------------------------------------------------------------------

    const cv::Size& GridDetector::input_resolution() const
    {
        return m_Settings.input_resolution;
    }

//---------------------------------------------------------------------------------------------------------------------

}
