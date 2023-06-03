//     *************************** LiveVisionKit ****************************
//     Copyright (C) 2022  Sebastian Di Marco (crowsinc.dev@gmail.com)
//
//     This program is free software: you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation, either version 3 of the License, or
//     (at your option) any later version.
//
//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.
//
//     You should have received a copy of the GNU General Public License
//     along with this program.  If not, see <https://www.gnu.org/licenses/>.
//     **********************************************************************

#include "FeatureDetector.hpp"

#include "Directives.hpp"
#include "Functions/Math.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	constexpr int FAST_MIN_THRESHOLD = 10;
	constexpr int FAST_MAX_THRESHOLD = 250;
	constexpr int FAST_THRESHOLD_STEP = 5.0f;
    constexpr int FAST_FEATURE_TOLERANCE = 150;


//---------------------------------------------------------------------------------------------------------------------
	
	FeatureDetector::FeatureDetector(const FeatureDetectorSettings& settings)
        : m_FeatureGrid(settings.feature_resolution),
          m_DetectionZones(settings.detection_zones),
          m_MinimumFeatureLoad(0),  // Set by configure()
          m_FASTFeatureTarget(0)    // Set by configure()
	{
        configure(settings);
	}

//---------------------------------------------------------------------------------------------------------------------

    void FeatureDetector::configure(const FeatureDetectorSettings& settings)
    {
        LVK_ASSERT(settings.detect_resolution.width > 0);
        LVK_ASSERT(settings.detect_resolution.height > 0);
        LVK_ASSERT(settings.detection_density > 1.0f);
        LVK_ASSERT_01(settings.detection_threshold);

        // Grids must be smaller or equal to resolution
        LVK_ASSERT(settings.feature_resolution.width <= settings.detect_resolution.width);
        LVK_ASSERT(settings.feature_resolution.height <= settings.detect_resolution.height);
        LVK_ASSERT(settings.detection_zones.width <= settings.detect_resolution.width);
        LVK_ASSERT(settings.detection_zones.height <= settings.detect_resolution.height);

        // Grids must evenly divide the resolution
        LVK_ASSERT(settings.detect_resolution.height % settings.detection_zones.height == 0);
        LVK_ASSERT(settings.detect_resolution.width % settings.detection_zones.width == 0);
        LVK_ASSERT(settings.detect_resolution.height % settings.feature_resolution.height == 0);
        LVK_ASSERT(settings.detect_resolution.width % settings.feature_resolution.width == 0);

        m_Settings = settings;

        const cv::Rect input_region({0,0}, settings.detect_resolution);
        const auto detect_zones = static_cast<float>(settings.detection_zones.area());
        const auto feature_blocks = static_cast<float>(settings.feature_resolution.area());

        m_MinimumFeatureLoad = static_cast<size_t>((feature_blocks * settings.detection_threshold) / detect_zones);
        m_FASTFeatureTarget = static_cast<size_t>((feature_blocks * settings.detection_density) / detect_zones);
        m_FASTFeatureBuffer.reserve(m_FASTFeatureTarget);

        m_FeatureGrid.clear();
        m_FeatureGrid.reshape(m_Settings.feature_resolution);
        m_FeatureGrid.align(input_region);

        m_DetectionZones.clear();
        m_DetectionZones.reshape(m_Settings.detection_zones);
        m_DetectionZones.align(input_region);

        construct_detection_zones();
    }

//---------------------------------------------------------------------------------------------------------------------

	void FeatureDetector::construct_detection_zones()
	{
        m_DetectionZones.align({{0,0}, m_Settings.detect_resolution});
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
				zone.fast_threshold = FAST_MIN_THRESHOLD;
				zone.propagations = 0;

                m_DetectionZones.place_at({c, r}, zone);
			}
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void FeatureDetector::detect(cv::UMat& frame, std::vector<cv::Point2f>& points)
	{
		LVK_ASSERT(frame.size() == m_Settings.detect_resolution);
		LVK_ASSERT(frame.type() == CV_8UC1);

		// Detect new features in the detection zones
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

                // Sift features into the feature grid.
				process_features(m_FASTFeatureBuffer, bounds.tl());

				// Dynamically adjust feature threshold to try meet the feature target next time
				if(m_FASTFeatureBuffer.size() > m_FASTFeatureTarget + FAST_FEATURE_TOLERANCE)
					fast_threshold = step(fast_threshold, FAST_MAX_THRESHOLD, FAST_THRESHOLD_STEP);
				else if(m_FASTFeatureBuffer.size() < m_FASTFeatureTarget - FAST_FEATURE_TOLERANCE)
					fast_threshold = step(fast_threshold, FAST_MIN_THRESHOLD, FAST_THRESHOLD_STEP);
            }
		}

		extract_features(points);
	}

//---------------------------------------------------------------------------------------------------------------------

	void FeatureDetector::process_features(const std::vector<cv::KeyPoint>& features, const cv::Point2f& offset)
	{
		// Process features into the feature grid, keeping only the best for each block.
		for(cv::KeyPoint feature : features)
		{
            LVK_ASSERT(m_FeatureGrid.within_bounds(feature.pt));

			feature.pt.x += offset.x;
			feature.pt.y += offset.y;

			auto& [curr_feature, propagated] = m_FeatureGrid[feature.pt];
			if(!propagated && curr_feature.response < feature.response)
                curr_feature = feature;
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void FeatureDetector::extract_features(std::vector<cv::Point2f>& feature_points) const
	{
		for(const auto& [coord, block] : m_FeatureGrid)
            feature_points.push_back(block.feature.pt);
    }

//---------------------------------------------------------------------------------------------------------------------

	void FeatureDetector::propagate(const std::vector<cv::Point2f>& points)
	{
		reset();

		// Add the propagated points onto the feature grid.
		for(const auto& point : points)
		{
			// Silently ignore points which are out of bounds.
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

	void FeatureDetector::reset()
	{
		for(auto& [coord, zone] : m_DetectionZones)
			zone.propagations = 0;

        m_FeatureGrid.clear();
	}

//---------------------------------------------------------------------------------------------------------------------

    size_t FeatureDetector::feature_capacity() const
    {
        return m_FeatureGrid.capacity();
    }

//---------------------------------------------------------------------------------------------------------------------

	cv::Size FeatureDetector::local_feature_size() const
	{
		return cv::Size(m_FeatureGrid.key_size());
	}

//---------------------------------------------------------------------------------------------------------------------

	cv::Size FeatureDetector::detection_zone_size() const
	{
		return cv::Size(m_DetectionZones.key_size());
	}

//---------------------------------------------------------------------------------------------------------------------

    float FeatureDetector::distribution_quality() const
    {
        return m_FeatureGrid.distribution_quality();
    }

//---------------------------------------------------------------------------------------------------------------------

	cv::Point2f FeatureDetector::distribution_centroid() const
	{
		return m_FeatureGrid.distribution_centroid<float>();
	}

//---------------------------------------------------------------------------------------------------------------------

}
