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
        : m_DetectionRegions({1,1}), // Set by configure()
          m_SuppressionGrid({1,1}),  // Set by configure()
          m_FASTDetector(cv::FastFeatureDetector::create(
              FAST_MIN_THRESHOLD, true,
              cv::FastFeatureDetector::TYPE_9_16
          ))
	{
        configure(settings);
	}

//---------------------------------------------------------------------------------------------------------------------

    void FeatureDetector::configure(const FeatureDetectorSettings& settings)
    {
        LVK_ASSERT(settings.detection_regions.width > 0 && settings.detection_regions.height > 0);
        LVK_ASSERT(settings.detection_regions.height <= settings.detection_resolution.height);
        LVK_ASSERT(settings.detection_regions.width <= settings.detection_resolution.width);
        LVK_ASSERT(settings.min_feature_density <= settings.max_feature_density);
        LVK_ASSERT(settings.min_feature_density > 0.0f);
        LVK_ASSERT(settings.accumulation_rate > 0.0f);
        LVK_ASSERT_01(settings.max_feature_density);
        LVK_ASSERT_01(settings.min_feature_density);

        const cv::Rect2f input_region({0,0}, settings.detection_resolution);

        // Create suppression grid
        m_SuppressionGrid.reshape(cv::Size2f(settings.detection_resolution) * settings.max_feature_density);
        m_SuppressionGrid.align(input_region);

        // Create FAST detection zones
        m_DetectionRegions.reshape(settings.detection_regions);
        m_DetectionRegions.align(input_region);
        construct_detection_regions();

        const auto max_regions = static_cast<float>(m_DetectionRegions.area());
        const auto max_region_features = static_cast<float>(m_SuppressionGrid.area()) / max_regions;
        const auto density_ratio = settings.min_feature_density / settings.max_feature_density;

        // Calculate min, max, and target feature loads for each detection zone.
        m_MinimumFeatureLoad = static_cast<size_t>(max_region_features * density_ratio);
        m_FASTFeatureTarget = static_cast<size_t>(settings.accumulation_rate * max_region_features);
        m_FASTFeatureBuffer.reserve(m_FASTFeatureTarget);

        m_Settings = settings;
    }

//---------------------------------------------------------------------------------------------------------------------

	void FeatureDetector::construct_detection_regions()
	{
        m_DetectionRegions.clear();

        // Construct the detection region grid
        const cv::Size2f region_size = m_DetectionRegions.key_size();
		for(size_t r = 0; r < m_DetectionRegions.rows(); r++)
		{
			for(size_t c = 0; c < m_DetectionRegions.cols(); c++)
			{
                FASTRegion region;
                region.bounds = cv::Rect2f(
                    static_cast<float>(c) * region_size.width,
                    static_cast<float>(r) * region_size.height,
                    region_size.width,
                    region_size.height
				);
                region.threshold = FAST_MIN_THRESHOLD;
                region.load = 0;

                m_DetectionRegions.place_at({c, r}, region);
			}
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	float FeatureDetector::detect(cv::InputArray frame, std::vector<cv::KeyPoint>& features)
	{
		LVK_ASSERT(frame.size() == m_Settings.detection_resolution);
        LVK_ASSERT(frame.isMat() || frame.isUMat());
		LVK_ASSERT(frame.type() == CV_8UC1);

		// Detect new features in the detection zones
		for(auto& [coord, region] : m_DetectionRegions)
		{
            auto& [bounds, threshold, load] = region;

			if(m_Settings.force_detection || load <= m_MinimumFeatureLoad)
			{
                m_FASTFeatureBuffer.clear();

                // Detect features in this region.
                m_FASTDetector->setThreshold(threshold);
                if(frame.isMat())
                    m_FASTDetector->detect(frame.getMat()(bounds), m_FASTFeatureBuffer);
                else
                    m_FASTDetector->detect(frame.getUMat()(bounds), m_FASTFeatureBuffer);

                // Process the features into the suppression grid for further non-maximal suppression.
                std::for_each(m_FASTFeatureBuffer.begin(), m_FASTFeatureBuffer.end(), [&](cv::KeyPoint& feature)
                {
                    // Update local region coordinate to global coordinate.
                    feature.pt += bounds.tl();

                    const auto& key = m_SuppressionGrid.key_of(feature.pt);
                    const auto& maximal_feature = m_SuppressionGrid.at_or(key, feature);

                    // Prefer maximal features
                    if(maximal_feature.response <= feature.response)
                        m_SuppressionGrid.emplace_at(key, feature);
                });

				// Dynamically adjust FAST threshold to try meet the feature target next time
				if(m_FASTFeatureBuffer.size() > m_FASTFeatureTarget + FAST_FEATURE_TOLERANCE)
					threshold = step(threshold, FAST_MAX_THRESHOLD, FAST_THRESHOLD_STEP);
				else if(m_FASTFeatureBuffer.size() < m_FASTFeatureTarget - FAST_FEATURE_TOLERANCE)
                    threshold = step(threshold, FAST_MIN_THRESHOLD, FAST_THRESHOLD_STEP);

            }
            load = 0; // Reset region
		}

        // Extract all the features from the suppression grid.
        for(const auto& [key, feature] : m_SuppressionGrid)
            features.push_back(feature);

        // Calculate the distribution quality of the points and clear the grid.
        float quality = m_SuppressionGrid.distribution_quality();
        m_SuppressionGrid.clear();

        return quality;
	}

//---------------------------------------------------------------------------------------------------------------------

	void FeatureDetector::propagate(const std::vector<cv::KeyPoint>& features)
	{
		for(const auto& feature : features)
		{
			// Silently ignore features which are out of bounds.
			if(const auto& key = m_SuppressionGrid.try_key_of(feature.pt); key.has_value())
			{
                m_SuppressionGrid.emplace_at(*key, feature);
                m_DetectionRegions[feature.pt].load++;
			}
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void FeatureDetector::reset()
	{
        m_SuppressionGrid.clear();
		for(auto& [coord, region] : m_DetectionRegions)
            region.load = 0;
	}

//---------------------------------------------------------------------------------------------------------------------

    size_t FeatureDetector::max_feature_capacity() const
    {
        return m_SuppressionGrid.area();
    }

//---------------------------------------------------------------------------------------------------------------------

	size_t FeatureDetector::min_feature_capacity() const
	{
        return m_MinimumFeatureLoad * m_DetectionRegions.area();
	}

//---------------------------------------------------------------------------------------------------------------------

}
