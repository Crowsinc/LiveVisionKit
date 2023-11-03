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

#include "Utility/Configurable.hpp"
#include "Data/SpatialMap.hpp"

namespace lvk
{

    struct FeatureDetectorSettings
    {
        cv::Size detection_resolution = {256, 256};
        cv::Size detection_regions = {2, 2};
        bool force_detection = false;

        float max_feature_density = 0.20f;
        float min_feature_density = 0.05f;
        float accumulation_rate = 2.0f;
    };


	class FeatureDetector final : public Configurable<FeatureDetectorSettings>
	{
	public:

		explicit FeatureDetector(const FeatureDetectorSettings& settings = {});

        void configure(const FeatureDetectorSettings& settings) override;

        float detect(cv::InputArray frame, std::vector<cv::KeyPoint>& features);

		void propagate(const std::vector<cv::KeyPoint>& features);

		void reset();


        size_t max_feature_capacity() const;

        size_t min_feature_capacity() const;

	private:

		struct FASTRegion
		{
            cv::Rect2f bounds;
            int threshold = 0;
            size_t load = 0;
		};

		void construct_detection_regions();

	private:
        SpatialMap<FASTRegion> m_DetectionRegions;
        SpatialMap<size_t> m_SuppressionGrid;
        std::vector<cv::KeyPoint> m_Features;

        std::vector<cv::KeyPoint> m_FASTFeatureBuffer;
        size_t m_FASTFeatureTarget = 0, m_MinimumFeatureLoad = 0;
        cv::Ptr<cv::FastFeatureDetector> m_FASTDetector = nullptr;
	};

}
