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

#include "FrameTracker.hpp"

#include "WarpField.hpp"
#include "Math/Math.hpp"
#include "Utility/Algorithm.hpp"
#include "Diagnostics/Directives.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	constexpr double METRIC_SMOOTHING_FACTOR = 0.05;
	constexpr double GOOD_DISTRIBUTION_QUALITY = 0.6;

//---------------------------------------------------------------------------------------------------------------------

	FrameTracker::FrameTracker(const FrameTrackerSettings& settings)
	{
        this->configure(settings);

		// Light sharpening kernel
		m_FilterKernel = cv::Mat({3, 3}, {
			0.0f, -0.5f,  0.0f,
		   -0.5f,  3.0f, -0.5f,
			0.0f, -0.5f,  0.0f
		});


		restart();
	}

//---------------------------------------------------------------------------------------------------------------------

    void FrameTracker::configure(const FrameTrackerSettings& settings)
    {
        LVK_ASSERT(settings.minimum_tracking_points >= 4);

        m_TrackedPoints.reserve(settings.detector.feature_capacity());
        m_MatchedPoints.reserve(settings.detector.feature_capacity());
        m_InlierStatus.reserve(settings.detector.feature_capacity());
        m_MatchStatus.reserve(settings.detector.feature_capacity());

        // If we are tracking motion with a resolution of 2x2 (Homography)
        // then tighten up the homography estimation parameters for global
        // motion. Otherwise, loosen them up to allow local motion through.
        if(settings.motion_resolution == WarpField::MinimumSize)
        {
            // For accurate Homography estimation
            m_USACParams.sampler = cv::SAMPLING_UNIFORM;
            m_USACParams.score = cv::SCORE_METHOD_MAGSAC;
            m_USACParams.loMethod = cv::LOCAL_OPTIM_SIGMA;
            m_USACParams.maxIterations = 100;
            m_USACParams.confidence = 0.99;
            m_USACParams.loIterations = 10;
            m_USACParams.loSampleSize = 20;
            m_USACParams.threshold = 4;
        }
        else
        {
            // For major outlier rejection
            m_USACParams.sampler = cv::SAMPLING_UNIFORM;
            m_USACParams.score = cv::SCORE_METHOD_MSAC;
            m_USACParams.loMethod = cv::LOCAL_OPTIM_INNER_LO;
            m_USACParams.maxIterations = 100;
            m_USACParams.confidence = 0.99;
            m_USACParams.loIterations = 10;
            m_USACParams.loSampleSize = 20;
            m_USACParams.threshold = 20;
        }

        m_Settings = settings;
    }

//---------------------------------------------------------------------------------------------------------------------

	void FrameTracker::restart()
	{
		m_FirstFrame = true;
		m_Settings.detector.reset();
	}

//---------------------------------------------------------------------------------------------------------------------

    std::optional<WarpField> FrameTracker::track(const cv::UMat& next_frame)
	{
		LVK_ASSERT(!next_frame.empty());
		LVK_ASSERT(next_frame.type() == CV_8UC1);

        // Reset the state to track the next frame
        m_TrackedPoints.clear();
        m_MatchedPoints.clear();

        // Mark the last tracked frame as the previous frame.
        std::swap(m_PrevFrame, m_NextFrame);

        // Import the next frame for tracking by scaling it to the tracking resolution.
        // We also enhance its sharpness to counteract the loss in quality from scaling.
        cv::resize(next_frame, m_NextFrame, tracking_resolution(), 0, 0, cv::INTER_AREA);
        cv::filter2D(m_NextFrame, m_NextFrame, m_NextFrame.type(), m_FilterKernel);

        if(m_FirstFrame)
        {
            m_FirstFrame = false;
            return std::nullopt;
        }

        // Detect tracking points in the previous frames. Note that this also
        // returns all the points that were propagated from the previous frame.
        m_Settings.detector.detect(m_PrevFrame, m_TrackedPoints);
        if(m_TrackedPoints.size() < m_Settings.minimum_tracking_points)
            return std::nullopt;

        m_DistributionQuality = exp_moving_average(
            m_DistributionQuality, m_Settings.detector.distribution_quality(), METRIC_SMOOTHING_FACTOR
        );


		// Match tracking points
		cv::calcOpticalFlowPyrLK(
			m_PrevFrame,
			m_NextFrame,
			m_TrackedPoints,
			m_MatchedPoints,
			m_MatchStatus,
			cv::noArray(),
			cv::Size(7, 7)
		);

        // TODO: filter by tracking error as well, only for very large errors tho
		fast_filter(m_TrackedPoints, m_MatchedPoints, m_MatchStatus);
        if(m_MatchedPoints.size() < m_Settings.minimum_tracking_points)
            return std::nullopt;


        // NOTE: We must have at least 4 points here.
        // NOTE: We force estimation of a affine homography if we do
        // not have good tracking point distribution, in order to avoid
        // creating global distortion based on dominant local motion.
        std::optional<Homography> motion;
        motion = Homography::Estimate(
            m_TrackedPoints,
            m_MatchedPoints,
            m_InlierStatus,
            m_USACParams,
            m_DistributionQuality < GOOD_DISTRIBUTION_QUALITY
        );

        // NOTE: We only propagate inlier points to  the GridDetector to
        // help ensure consistency between subsequent motion estimations.
        // Additionally, the GridDetector doesn't detect new points if the
        // propagated points meet the detection load. This means that outliers
        // are naturally removed from the tracking point set until we have lost
        // too many inliers, and the GridDetector has to detect new points.

        const size_t total_tracking_points = m_TrackedPoints.size();
        fast_filter(m_TrackedPoints, m_MatchedPoints, m_InlierStatus);
        m_Settings.detector.propagate(m_MatchedPoints);

        m_FrameStability = exp_moving_average(
            m_FrameStability,
            static_cast<double>(m_MatchedPoints.size()) / static_cast<double>(total_tracking_points),
            METRIC_SMOOTHING_FACTOR
        );

        WarpField motion_field(m_Settings.motion_resolution);
        if(m_Settings.motion_resolution != WarpField::MinimumSize)
        {
            const cv::Rect region({0,0}, tracking_resolution());
            motion_field.fit_points(region, m_TrackedPoints, m_MatchedPoints, motion);
        }
        else motion_field.set_to(*motion, tracking_resolution());

        // We must scale the motion to match the original frame size.
        const cv::Size2f frame_scale = next_frame.size();
        const cv::Size2f tracking_scale = tracking_resolution();

        motion_field *= cv::Vec2f{
            frame_scale.width / tracking_scale.width,
            frame_scale.height / tracking_scale.height
        };

        return std::move(motion_field);
	}

//---------------------------------------------------------------------------------------------------------------------

	double FrameTracker::frame_stability() const
	{
		return m_FrameStability;
	}

//---------------------------------------------------------------------------------------------------------------------

    double FrameTracker::tracking_quality() const
    {
        return m_DistributionQuality;
    }

//---------------------------------------------------------------------------------------------------------------------

    const cv::Size& FrameTracker::motion_resolution() const
    {
        return m_Settings.motion_resolution;
    }

//---------------------------------------------------------------------------------------------------------------------

    const cv::Size& FrameTracker::tracking_resolution() const
    {
        return m_Settings.detector.input_resolution();
    }

//---------------------------------------------------------------------------------------------------------------------

	const std::vector<cv::Point2f>& FrameTracker::tracking_points() const
	{
		return m_MatchedPoints;
	}

//---------------------------------------------------------------------------------------------------------------------

}
