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
#include "Math/WarpMesh.hpp"
#include "Utility/Configurable.hpp"

namespace lvk
{

    struct FrameTrackerSettings : public FeatureDetectorSettings
    {
        cv::Size motion_resolution = {2, 2};

        // Local Motion Constraints
        bool track_local_motions = false;
        float local_smoothing = 1.5f;

        // Robustness Constraints
        size_t min_motion_samples = 75;
        float min_motion_quality = 0.20f;
        float stability_threshold = 0.30f;
    };

	class FrameTracker final : public Configurable<FrameTrackerSettings>
	{
	public:

		explicit FrameTracker(const FrameTrackerSettings& settings = {});

        void configure(const FrameTrackerSettings& settings) override;

		std::optional<WarpMesh> track(const cv::UMat& next_frame);

		void restart();

        float tracking_quality() const;

        const cv::Size& motion_resolution() const;

        const cv::Size& tracking_resolution() const;

		const std::vector<cv::Point2f>& tracking_points() const;

        void draw_trackers(cv::UMat& dst, const cv::Scalar& color, const int size = 10, const int thickness = 3) const;

    private:

        WarpMesh estimate_local_motions(
            const cv::Rect2f& region,
            const Homography& global_transform,
            const std::vector<cv::Point2f>& tracked_points,
            const std::vector<cv::Point2f>& matched_points
        );

    private:
        bool m_FrameInitialized = false;
        cv::UMat m_PreviousFrame, m_CurrentFrame;

        FeatureDetector m_FeatureDetector;
        std::vector<cv::Point2f> m_TrackedPoints, m_MatchedPoints;

        float m_TrackingQuality;
        cv::Rect2f m_TrackingRegion;
		std::vector<uint8_t> m_MatchStatus;
        cv::Ptr<cv::SparsePyrLKOpticalFlow> m_OpticalTracker = nullptr;

        cv::UsacParams m_USACParams;
        std::vector<uchar> m_InlierStatus;
	};

}
