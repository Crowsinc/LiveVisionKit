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

#pragma once

#include <opencv2/opencv.hpp>

#include "FeatureDetector.hpp"
#include "Math/WarpField.hpp"
#include "Utility/Configurable.hpp"

namespace lvk
{

    struct FrameTrackerSettings : public FeatureDetectorSettings
    {
        cv::Size motion_resolution = {2, 2};

        // Motion Estimation Constraints
        float stability_threshold = 0.3f;
        float uniformity_threshold = 0.1f;
        size_t sample_size_threshold = 40;
    };

	class FrameTracker final : public Configurable<FrameTrackerSettings>
	{
	public:

		explicit FrameTracker(const FrameTrackerSettings& settings = {});

        void configure(const FrameTrackerSettings& settings) override;

		std::optional<WarpField> track(const cv::UMat& next_frame);

		void restart();

        float scene_stability() const;

        float scene_uniformity() const;

        const cv::Size& motion_resolution() const;

        const cv::Size& tracking_resolution() const;

		const std::vector<cv::Point2f>& tracking_points() const;

        void draw_trackers(cv::UMat& dst, const cv::Scalar& color, const int size = 10, const int thickness = 3);

    private:

        std::nullopt_t abort_tracking();

    private:
        FeatureDetector m_FeatureDetector;
        std::vector<cv::Point2f> m_TrackedPoints, m_MatchedPoints;

        cv::Rect2f m_TrackingRegion;
		std::vector<uint8_t> m_MatchStatus;
        cv::Ptr<cv::SparsePyrLKOpticalFlow> m_OpticalTracker = nullptr;

        bool m_PastFrameLoaded = false;
        cv::UMat m_PreviousFrame, m_CurrentFrame;

        cv::UsacParams m_USACParams;
        std::vector<uchar> m_InlierStatus;
        float m_Stability = 0.0f, m_Uniformity = 0.0f;

	};

}
