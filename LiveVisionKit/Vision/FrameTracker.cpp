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

    constexpr auto HOMOGRAPHY_DISTRIBUTION_THRESHOLD = 0.6f;
    constexpr auto STABILITY_CONTINUITY_THRESHOLD = 0.3f;
    constexpr auto USAC_NOISE_TOLERANCE = 5.0f;
    constexpr auto USAC_MAX_ITERS = 50;

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
        m_USACParams.threshold = USAC_NOISE_TOLERANCE;
        m_USACParams.maxIterations = USAC_MAX_ITERS;
        m_USACParams.score = cv::SCORE_METHOD_MAGSAC;
        m_USACParams.score = cv::SCORE_METHOD_LMEDS;
        m_USACParams.final_polisher = cv::MAGSAC;
        m_USACParams.final_polisher_iterations = 5;
        m_USACParams.loMethod = cv::LOCAL_OPTIM_SIGMA;
        m_USACParams.loIterations = 10;
        m_USACParams.loSampleSize = 20;

		restart();
	}

//---------------------------------------------------------------------------------------------------------------------

    void FrameTracker::configure(const FrameTrackerSettings& settings)
    {
        LVK_ASSERT(settings.max_local_increase > 0.0f);
        LVK_ASSERT(settings.min_motion_samples >= 4);
        LVK_ASSERT_01(settings.min_motion_quality);
        LVK_ASSERT_01(settings.max_local_variance);

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
        if(!m_FrameInitialized || m_CurrentFrame.size() != m_PreviousFrame.size())
        {
            m_FrameInitialized = true;
            return std::nullopt;
        }


        // Detect tracking points in the current frame.
        const auto sample_distribution = m_FeatureDetector.detect(m_CurrentFrame, m_TrackedPoints);
        if(m_TrackedPoints.size() < m_Settings.min_motion_samples)
            return std::nullopt;

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
        fast_filter(m_TrackedPoints, m_MatchedPoints, m_MatchStatus);
        const auto samples = m_MatchedPoints.size();


        // Define tracking quality from the match rate and sample distribution.
        m_TrackingQuality = ratio_of<float>(samples, m_MatchStatus.size()) * sample_distribution;
        if(m_TrackingQuality < m_Settings.min_motion_quality || samples < m_Settings.min_motion_samples)
        {
            m_MatchedPoints.clear();
            return std::nullopt;
        }


        // Estimate the global motion of the frame.
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
            // If the points aren't well distributed, fall back to an affine
            // homography to avoid creating perspectivity-based distortions.
            global_motion = Homography::FromAffineMatrix(
                cv::estimateAffinePartial2D(
                    m_TrackedPoints,
                    m_MatchedPoints,
                    m_InlierStatus,
                    cv::RANSAC,
                    USAC_NOISE_TOLERANCE,
                    USAC_MAX_ITERS
                )
            );
        }

        // Scene stability is defined as the inlier ratio of global motion.
        m_SceneStability = ratio_of<uchar>(m_InlierStatus, 1);

        // A sudden low spike in stability often indicates a discontinuity.
        if(m_Settings.discontinuity_detection && m_SceneStability < STABILITY_CONTINUITY_THRESHOLD)
        {
            m_MatchedPoints.clear();
            return std::nullopt;
        }

        WarpField motion = should_use_homography() ? WarpField(global_motion, m_Settings.detection_resolution)
            : estimate_local_motions(m_TrackingRegion, global_motion, m_TrackedPoints, m_MatchedPoints, m_InlierStatus);

        // Filter inliers so we're left with only high quality points,
        // then propagate them so that they're re-used in the detector.
        fast_filter(m_MatchedPoints, m_InlierStatus);
        m_FeatureDetector.propagate(m_MatchedPoints);

        return motion;
	}

//---------------------------------------------------------------------------------------------------------------------

    bool FrameTracker::should_use_homography() const
    {
        return m_Settings.motion_resolution == WarpField::MinimumSize;
    }

//---------------------------------------------------------------------------------------------------------------------


    WarpField FrameTracker::estimate_local_motions(
        const cv::Rect2f& region,
        const Homography& global_transform,
        const std::vector<cv::Point2f>& tracked_points,
        const std::vector<cv::Point2f>& matched_points,
        std::vector<uint8_t>& inlier_status
    )
    {
        LVK_ASSERT(tracked_points.size() == matched_points.size());
        LVK_ASSERT(inlier_status.size() == tracked_points.size());

        // Initialize frame motion to the global transform (e.g. background motion).
        WarpField frame_motion(global_transform, m_Settings.detection_resolution, m_Settings.motion_resolution);
        cv::Mat& global_field = frame_motion.offsets();

        // Create a motion accumulation grid whose cells are centered on the field points.
        const cv::Size2f field_size = m_Settings.motion_resolution;
        const cv::Size2f vertex_area = region.size() / (field_size - 1.0f);
        const VirtualGrid grid(field_size, {region.tl() * (vertex_area / 2.0f), field_size * vertex_area});

        const auto local_norm_factor = 1.0f / region.size();
        const auto local_angle_limit = m_Settings.max_local_variance * std::numbers::pi;

        // Accumulate all the local motion residuals using the grid.
        cv::Mat accumulator(m_Settings.motion_resolution, CV_32FC3, cv::Scalar::zeros());
        for(size_t i = 0; i < tracked_points.size(); i++)
        {
            const cv::Point2f& src_point = tracked_points[i];
            const cv::Point2f& dst_point = matched_points[i];

            if(const auto coord = grid.key_of(dst_point); grid.test_point(dst_point))
            {
                const auto global_motion = global_field.at<cv::Point2f>(coord);
                const auto local_motion = (src_point - dst_point) * local_norm_factor;
                const auto l1 = local_motion.dot(local_motion), l2 = global_motion.dot(global_motion);

                // Only consider local motions that are similar to the global motion in both
                // magnitude and direction. Motions which arise from parallax or depth changes
                // should have similar direction, while foreground motions may vary drastically.
                const bool similar_direction = std::abs(angle_of(local_motion, global_motion)) <= local_angle_limit;
                const bool similar_magnitude = l1 <= m_Settings.max_local_increase * l2;

                if(uint8_t& inlier = inlier_status[i]; similar_direction && similar_magnitude)
                {
                    auto& [dx, dy, count] = accumulator.at<cv::Point3f>(coord);
                    dx += local_motion.x - global_motion.x;
                    dy += local_motion.y - global_motion.y;
                    count += 1.0f;
                    inlier = 1;
                }
            }
        }

        // Spread out the accumulated residuals using an un-normalized box filter.
        cv::boxFilter(accumulator, accumulator, -1, {3, 3}, {-1,-1}, false, cv::BORDER_REFLECT_101);

        // Flatten out accumulated motions into average local residuals.
        cv::Mat residuals(m_Settings.motion_resolution, CV_32FC2);
        residuals.forEach<cv::Point2f>([&](cv::Point2f& residual, const int coord[]){
            const auto& [dx, dy, count] = accumulator.at<cv::Point3f>(coord[0], coord[1]);

            // NOTE: clamp count to 1 and above to avoid dividing by zero.
            const auto weight = 1.0f / std::max(count, 1.0f);
            residual.x = dx * weight;
            residual.y = dy * weight;
        });

        // Spatially smooth and combine the residuals.
        cv::medianBlur(residuals, residuals, 3);
        global_field += residuals;

        return frame_motion;
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
