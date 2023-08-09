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
#include "Timing/Stopwatch.hpp"

#include "Eigen/Geometry"
#include "Eigen/Sparse"

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
        LVK_ASSERT(settings.motion_resolution.height >= WarpField::MinimumSize.height);
        LVK_ASSERT(settings.motion_resolution.width >= WarpField::MinimumSize.width);
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

        // A sudden low spike in global inlier ratio often indicates a discontinuity.
        if(ratio_of<uchar>(m_InlierStatus, 1) < m_Settings.continuity_threshold)
        {
            m_MatchedPoints.clear();
            return std::nullopt;
        }

        // Filter inliers so we're left with only high quality points,
        // then propagate them so that they're re-used in the detector.
//        fast_filter(m_TrackedPoints, m_MatchedPoints, m_InlierStatus);
        m_FeatureDetector.propagate(m_MatchedPoints);

        return should_use_homography() ? WarpField(global_motion, m_Settings.detection_resolution)
            : estimate_local_motions(m_TrackingRegion, global_motion, m_TrackedPoints, m_MatchedPoints);
	}

//---------------------------------------------------------------------------------------------------------------------

    bool FrameTracker::should_use_homography() const
    {
        return m_Settings.motion_resolution == WarpField::MinimumSize;
    }

//---------------------------------------------------------------------------------------------------------------------

    // TODO: similarity transform.
    // TODO: PathSmoother make min smoothing factor be based on path prediction frames.
    // TODO: make motion resolution only be power of 2 (universally)

    WarpField FrameTracker::estimate_local_motions(
        const cv::Rect2f& region,
        const Homography& global_transform,
        const std::vector<cv::Point2f>& tracked_points,
        const std::vector<cv::Point2f>& matched_points
    )
    {
        LVK_ASSERT(tracked_points.size() == matched_points.size());

        // TODO: remove
        thread_local Stopwatch s(1000);
        s.start();

        // Create a grid partition for the motion warp mesh.
        const auto mesh_size = m_Settings.motion_resolution;
        const auto grid_size = mesh_size - cv::Size(1, 1);

        const VirtualGrid mesh_grid(mesh_size, cv::Rect2f(
            region.tl(), (cv::Size2f(mesh_size) / cv::Size2f(grid_size)) * region.size()
        ));

        // Initialize linear system to optimize the mesh
        const int equation_count = 2 * (mesh_size.area() + matched_points.size() + grid_size.area());
        Eigen::SparseMatrix<float> A(equation_count, 2 * mesh_size.area());
        Eigen::VectorXf b = Eigen::VectorXf::Zero(equation_count);
        Eigen::VectorXf global_mesh(2 * mesh_size.area());

        int constraint_offset = 0;
        std::vector<Eigen::Triplet<float>> values;

        // TODO: update
        values.reserve(b.rows() + 8 * matched_points.size() + 14 * grid_size.area());

        // Add global motion mesh constraints.
        const auto inverse_transform = global_transform.invert();
        mesh_grid.for_each_aligned([&](const int index, const cv::Point2f& aligned_coord){
            const auto global_mesh_coord = inverse_transform * aligned_coord;

            const int x_index = 2 * index;
            values.emplace_back(constraint_offset, x_index, 1.0f);
            global_mesh(x_index) = global_mesh_coord.x;
            b(constraint_offset) = global_mesh_coord.x;
            constraint_offset++;

            const int y_index = x_index + 1;
            values.emplace_back(constraint_offset, y_index, 1.0f);
            global_mesh(y_index) = global_mesh_coord.y;
            b(constraint_offset) = global_mesh_coord.y;
            constraint_offset++;


        });

        // Add local motion mesh constraints.
        for(size_t i = 0; i < matched_points.size(); i++)
        {
            const cv::Point2f& src_point = tracked_points[i];
            const cv::Point2f& dst_point = matched_points[i];

            // k00 ---- k10
            //  |        |
            //  |        |
            // k01 ---- k11

            cv::Point k00 = mesh_grid.key_of(src_point);
            k00.x = std::clamp(k00.x, 0, grid_size.width);
            k00.y = std::clamp(k00.y, 0, grid_size.height);
            const cv::Point k11 = k00 + 1;

            // Get indices of the mesh vertices
            const int i00 = 2 * static_cast<int>(mesh_grid.key_to_index(k00));
            const int i11 = 2 * static_cast<int>(mesh_grid.key_to_index(k11));
            const int i10 = i00 + 2;
            const int i01 = i11 - 2;

            // Get barycentric weights of the src point in its cell,
            // we want these weights to hold in the warped motion mesh.
            const auto w = barycentric_rect(
                {mesh_grid.key_to_point(k00), mesh_grid.key_to_point(k11)}, src_point
            );

            // TODO: make dynamic
            constexpr float a = 0.75f;

            values.emplace_back(constraint_offset, i00, w[0] * a);
            values.emplace_back(constraint_offset, i01, w[1] * a);
            values.emplace_back(constraint_offset, i11, w[2] * a);
            values.emplace_back(constraint_offset, i10, w[3] * a);
            b(constraint_offset) = dst_point.x * a;
            constraint_offset++;

            values.emplace_back(constraint_offset, i00 + 1, w[0] * a);
            values.emplace_back(constraint_offset, i01 + 1, w[1] * a);
            values.emplace_back(constraint_offset, i11 + 1, w[2] * a);
            values.emplace_back(constraint_offset, i10 + 1, w[3] * a);
            b(constraint_offset) = dst_point.y * a;
            constraint_offset++;
        }

        // Add mesh similarity constraints.
        mesh_grid.for_each([&](const int index, const cv::Point& coord) {

            if(coord.x == mesh_size.width - 1 || coord.y == mesh_size.height - 1)
                return;

            // TODO: make dynamic
            const float w = 0.5f;

            // V00 ---- V10
            //  |  .     |
            //  |     .  |
            // V01 ---- V11

            const int i00 = 2 * index;
            const int i01 = i00 + 2 * mesh_size.width;
            const int i10 = i00 + 2;
            const int i11 = i01 + 2;

            const cv::Point2f V00(global_mesh(i00), global_mesh(i00 + 1));
            const cv::Point2f V01(global_mesh(i01), global_mesh(i01 + 1));
            const cv::Point2f V11(global_mesh(i11), global_mesh(i11 + 1));
            const cv::Point2f V10(global_mesh(i10), global_mesh(i10 + 1));

            // Lower Triangle
            const cv::Point2f V1 = V00 - V01;
            const cv::Point2f R1 = V11 - V01;
            const float l1 = 1.0f/R1.dot(R1);

            const float u1 = (R1.x * V1.x + R1.y * V1.y) * l1;
            const float v1 = (R1.y * V1.x - R1.x * V1.y) * l1;

            // Upper Triangle
            const cv::Point2f V2 = V00 - V10;
            const cv::Point2f R2 = V11 - V10;
            const auto l2 = 1.0f/R2.dot(R2);

            const float u2 = (R2.x * V2.x + R2.y * V2.y) * l2;
            const float v2 = (R2.y * V2.x - R2.x * V2.y) * l2;


            values.emplace_back(constraint_offset, i00, -2.0f * w);
            values.emplace_back(constraint_offset, i11, (u1 + u2) * w);
            values.emplace_back(constraint_offset, i11 + 1, (v1 - v2) * w);
            values.emplace_back(constraint_offset, i01, (1.0f - u1) * w);
            values.emplace_back(constraint_offset, i01 + 1, -v1 * w);
            values.emplace_back(constraint_offset, i10, (1.0f - u2) * w);
            values.emplace_back(constraint_offset, i10 + 1, v2 * w);
            constraint_offset++;


            values.emplace_back(constraint_offset, i00 + 1, -2.0f * w);
            values.emplace_back(constraint_offset, i11, (v2 - v1) * w);
            values.emplace_back(constraint_offset, i11 + 1, (u1 + u2) * w);
            values.emplace_back(constraint_offset, i01, v1 * w);
            values.emplace_back(constraint_offset, i01 + 1, (1.0f - u1) * w);
            values.emplace_back(constraint_offset, i10, -v2 * w);
            values.emplace_back(constraint_offset, i10 + 1, (1.0f - u2) * w);
            constraint_offset++;
        });

        A.setFromTriplets(values.begin(), values.end());

        // Solve the system to get the optimal motion mesh.
        Eigen::LeastSquaresConjugateGradient<Eigen::SparseMatrix<float>> solver(A);
        Eigen::VectorXf results = solver.solveWithGuess(b, global_mesh);

        cv::Mat mesh(m_Settings.motion_resolution, CV_32FC2, results.data());
        mesh_grid.for_each_aligned([&](const int index, const cv::Point2f& aligned_coord){
            auto& vertex = mesh.at<cv::Point2f>(index);
            vertex = (vertex - aligned_coord) / region.size();
        });

        s.stop();
        std::cout << s.average().milliseconds() << "ms\n";

        // NOTE: Mesh memory is owned by Eigen.
        return WarpField(mesh, true, true);
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

    void FrameTracker::draw_trackers(cv::UMat& dst, const cv::Scalar& colour, const int size, const int thickness) const
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
