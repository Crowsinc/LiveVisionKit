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

#include "FrameTracker.hpp"

#include "Directives.hpp"
#include "Math/Homography.hpp"
#include "Functions/Container.hpp"
#include "Functions/Extensions.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

    // NOTE: if you set the window size to less than 9x9, OpenCV will
    // run it on the CPU, leading to a large increase in CPU usage in
    // exchange for it running much faster than the GPU version.
    const cv::Size OPTICAL_TRACKER_WIN_SIZE = {11, 11};
    constexpr auto OPTICAL_TRACKER_PYR_LEVELS = 3;
    constexpr auto OPTICAL_TRACKER_MAX_ITERS = 5;

    constexpr auto USAC_MAX_ITERS = 50;
    constexpr auto USAC_THRESHOLD = 5.0f;

    constexpr auto STABILITY_CONTINUITY_THRESHOLD = 0.3f;
    constexpr auto HOMOGRAPHY_DISTRIBUTION_THRESHOLD = 0.6f;

//---------------------------------------------------------------------------------------------------------------------

	FrameTracker::FrameTracker(const FrameTrackerSettings& settings)
        : m_OpticalTracker(cv::SparsePyrLKOpticalFlow::create(
              OPTICAL_TRACKER_WIN_SIZE, OPTICAL_TRACKER_PYR_LEVELS,
              cv::TermCriteria(
                  cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                  OPTICAL_TRACKER_MAX_ITERS, 0.01
              ),
              cv::OPTFLOW_USE_INITIAL_FLOW // NOTE: we do not clear matched point state.
          ))
	{
        configure(settings);

        // Set USAC params for accurate Homography estimation
        m_USACParams.confidence = 0.99;
        m_USACParams.threshold = USAC_THRESHOLD;
        m_USACParams.maxIterations = USAC_MAX_ITERS;
        m_USACParams.sampler = cv::SAMPLING_UNIFORM;
        m_USACParams.score = cv::SCORE_METHOD_MAGSAC;

        m_USACParams.loMethod = cv::LOCAL_OPTIM_SIGMA;
        m_USACParams.loIterations = 10;
        m_USACParams.loSampleSize = 20;

        m_USACParams.final_polisher = cv::MAGSAC;
        m_USACParams.final_polisher_iterations = 5;

		restart();
	}

//---------------------------------------------------------------------------------------------------------------------

    void FrameTracker::configure(const FrameTrackerSettings& settings)
    {
        LVK_ASSERT(settings.min_motion_samples >= 4);
        LVK_ASSERT_01(settings.min_motion_quality);

        m_FeatureDetector.configure(settings);
        m_MatchStatus.reserve(m_FeatureDetector.max_feature_capacity());
        m_InlierStatus.reserve(m_FeatureDetector.max_feature_capacity());
        m_TrackedPoints.reserve(m_FeatureDetector.max_feature_capacity());
        m_MatchedPoints.reserve(m_FeatureDetector.max_feature_capacity());
        m_TrackingRegion = cv::Rect2f({0,0}, settings.detection_resolution);

        // We need to reset the detector and rescale the last frame if the resolution changed.
        if(settings.detection_resolution != m_Settings.detection_resolution && m_FrameInitialized)
        {
            m_MatchedPoints.clear();
            m_FeatureDetector.reset();
            cv::resize(m_CurrentFrame, m_CurrentFrame, m_Settings.detection_resolution, 0, 0, cv::INTER_LINEAR);
        }

        m_Settings = settings;
    }

//---------------------------------------------------------------------------------------------------------------------

	void FrameTracker::restart()
	{
        m_SceneStability = 0.0f;
        m_TrackingQuality = 0.0f;
        m_TrackedPoints.clear();
        m_MatchedPoints.clear();
        m_FeatureDetector.reset();
        m_FrameInitialized = false;
	}

//---------------------------------------------------------------------------------------------------------------------

    std::optional<WarpField> FrameTracker::track(const cv::UMat& next_frame)
	{
		LVK_ASSERT(!next_frame.empty() && next_frame.type() == CV_8UC1);

        // Reset tracking state
        m_TrackedPoints.clear();
        m_SceneStability = 0.0f;
        m_TrackingQuality = 0.0f;

        // Advance time and import the next frame.
        std::swap(m_PreviousFrame, m_CurrentFrame);
        cv::resize(next_frame, m_CurrentFrame, m_Settings.detection_resolution, 0, 0, cv::INTER_AREA);

        // We need at least two frames for tracking.
        if(!m_FrameInitialized)
        {
            m_FrameInitialized = true;
            return std::nullopt;
        }

        // Detect tracking points in the current frame.
        const auto sample_distribution = m_FeatureDetector.detect(m_CurrentFrame, m_TrackedPoints);
        if(m_TrackedPoints.size() < m_Settings.min_motion_samples)
            return abort_tracking();

        // Create initial guesses for matched points.
        if(m_MatchedPoints.size() != m_TrackedPoints.size())
            m_MatchedPoints = m_TrackedPoints;


		// Match tracking points.
        m_OpticalTracker->calc(
            m_PreviousFrame,
            m_CurrentFrame,
            m_TrackedPoints,
            m_MatchedPoints,
            m_MatchStatus
        );

        // Filter out any points that weren't matched or are out of bounds.
        for(int i = static_cast<int>(m_MatchStatus.size()) - 1; i >= 0; i--)
        {
            if(!m_MatchStatus[i] || !m_TrackingRegion.contains(m_MatchedPoints[i]))
            {
                fast_erase(m_TrackedPoints, i);
                fast_erase(m_MatchedPoints, i);
            }
        }
        const auto samples = m_MatchedPoints.size();


        // Calculate quality using match rate and distribution.
        m_TrackingQuality = ratio_of<float>(samples, m_MatchStatus.size()) * sample_distribution;
        if(m_TrackingQuality < m_Settings.min_motion_quality || samples < m_Settings.min_motion_samples)
            return abort_tracking();


        // Estimate the global motion of the frame. If we don't have a
        // high enough sample distribution, then fall back to an affine
        // homography to avoid creating perspectivity-based distortions.
        Homography global_motion;
        if(sample_distribution > HOMOGRAPHY_DISTRIBUTION_THRESHOLD)
        {
            global_motion = cv::findHomography(
                m_TrackedPoints,
                m_MatchedPoints,
                m_InlierStatus,
                m_USACParams
            );
        }
        else
        {
            global_motion = Homography::FromAffineMatrix(
                cv::estimateAffinePartial2D(
                    m_TrackedPoints,
                    m_MatchedPoints,
                    m_InlierStatus,
                    cv::RANSAC,
                    USAC_THRESHOLD,
                    USAC_MAX_ITERS
                )
            );
        }


        // Filter out any outliers and propagate the inliers.
        fast_filter(m_MatchedPoints, m_TrackedPoints, m_InlierStatus);
        m_FeatureDetector.propagate(m_MatchedPoints);


        // Define scene stability from the inlier ratio. Low scene stability
        // indicates a lot of noise or moving objects. A sudden low spike in
        // scene stability often indicates a discontinuity between the frames.
        m_SceneStability = ratio_of<float>(m_MatchedPoints.size(), samples);
        if(m_SceneStability < STABILITY_CONTINUITY_THRESHOLD)
            return abort_tracking();


        // TODO: replace with a new algorithm.
        // If our motion resolution is greater than 2x2, create a vector field.
        WarpField local_motion(m_Settings.motion_resolution);
        if(m_Settings.motion_resolution != WarpField::MinimumSize)
        {
            local_motion.fit_points(
                m_TrackingRegion,
                global_motion,
                m_TrackedPoints,
                m_MatchedPoints
            );
        }
        else local_motion.set_to(global_motion, m_Settings.detection_resolution);

        return std::move(local_motion);
	}

//---------------------------------------------------------------------------------------------------------------------

    std::nullopt_t FrameTracker::abort_tracking()
    {
        m_MatchedPoints.clear();
        return std::nullopt;
    }

//---------------------------------------------------------------------------------------------------------------------

    float FrameTracker::scene_stability() const
	{
		return m_SceneStability;
	}

//---------------------------------------------------------------------------------------------------------------------

    float FrameTracker::tracking_quality() const
    {
        return m_TrackingQuality;
    }

//---------------------------------------------------------------------------------------------------------------------

    const cv::Size& FrameTracker::motion_resolution() const
    {
        return m_Settings.motion_resolution;
    }

//---------------------------------------------------------------------------------------------------------------------

    const cv::Size& FrameTracker::tracking_resolution() const
    {
        return m_Settings.detection_resolution;
    }

//---------------------------------------------------------------------------------------------------------------------

	const std::vector<cv::Point2f>& FrameTracker::tracking_points() const
	{
		return m_MatchedPoints;
	}

//---------------------------------------------------------------------------------------------------------------------

    void FrameTracker::draw_trackers(cv::UMat& dst, const cv::Scalar& colour, const int size, const int thickness)
    {
        LVK_ASSERT(thickness > 0);
        LVK_ASSERT(size > 0);

        draw_crosses(
            dst,
            m_MatchedPoints,
            colour,
            size, 4,
            cv::Size2f(dst.size()) / cv::Size2f(m_Settings.detection_resolution)
        );
    }

//---------------------------------------------------------------------------------------------------------------------

}
