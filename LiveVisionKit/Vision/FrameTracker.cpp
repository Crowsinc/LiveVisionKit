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
    constexpr auto USAC_NOISE_TOLERANCE = 10.0f;
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
        m_USACParams.sampler = cv::SAMPLING_UNIFORM;
        m_USACParams.score = cv::SCORE_METHOD_MAGSAC;
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
        LVK_ASSERT(settings.motion_resolution.height >= WarpMesh::MinimumSize.height);
        LVK_ASSERT(settings.motion_resolution.width >= WarpMesh::MinimumSize.width);
        LVK_ASSERT(settings.local_smoothing > 0.0f);
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
        m_TrackingQuality = 0.0f;
        m_TrackedPoints.clear();
        m_MatchedPoints.clear();
        m_FeatureDetector.reset();
        m_FrameInitialized = false;
	}

//---------------------------------------------------------------------------------------------------------------------

    std::optional<WarpMesh> FrameTracker::track(const cv::UMat& next_frame)
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

        // Filter outliers so we're left with only high quality points,
        // then propagate them so that they're re-used in the detector.
        fast_filter(m_TrackedPoints, m_MatchedPoints, m_InlierStatus);
        m_FeatureDetector.propagate(m_MatchedPoints);

        if(m_Settings.track_local_motions)
            return estimate_local_motions(m_TrackingRegion, global_motion, m_TrackedPoints, m_MatchedPoints);
        else
            return WarpMesh(global_motion, m_Settings.detection_resolution, m_Settings.motion_resolution);
	}

//---------------------------------------------------------------------------------------------------------------------

    WarpMesh FrameTracker::estimate_local_motions(
        const cv::Rect2f& region,
        const Homography& global_transform,
        const std::vector<cv::Point2f>& tracked_points,
        const std::vector<cv::Point2f>& matched_points
    )
    {
        LVK_ASSERT(tracked_points.size() == matched_points.size());

        const auto mesh_size = m_Settings.motion_resolution;
        const auto grid_size = mesh_size - cv::Size(1, 1);

        // Create a partitioned grid for the motion warp mesh.
        const VirtualGrid mesh_grid(mesh_size, cv::Rect2f(
            region.tl(), (cv::Size2f(mesh_size) / cv::Size2f(grid_size)) * region.size()
        ));

        // Initialize linear system to optimize the mesh
        const auto constraints = 2 * matched_points.size() + 2 * mesh_size.area() + 4 * grid_size.area();
        Eigen::SparseMatrix<float> A(static_cast<int>(constraints), 2 * mesh_size.area());
        Eigen::VectorXf b = Eigen::VectorXf::Zero(A.rows());

        int constraint_offset = 0;
        std::vector<Eigen::Triplet<float>> sparse_values;
        sparse_values.reserve(b.rows() + 8 * matched_points.size() + 20 * grid_size.area());

        // Add global motion mesh constraints.
        Eigen::VectorXf global_mesh(2 * mesh_size.area());
        mesh_grid.for_each_aligned([&](const int index, const cv::Point2f& aligned_coord){
            const auto global_mesh_coord = global_transform * aligned_coord;

            const int x_index = 2 * index;
            sparse_values.emplace_back(constraint_offset, x_index, 1.0f);
            global_mesh(x_index) = global_mesh_coord.x;
            b(constraint_offset) = global_mesh_coord.x;
            constraint_offset++;

            const int y_index = x_index + 1;
            sparse_values.emplace_back(constraint_offset, y_index, 1.0f);
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
            cv::Scalar w = barycentric_rect(
                {mesh_grid.key_to_point(k00), mesh_grid.key_to_point(k11)}, src_point
            );

            sparse_values.emplace_back(constraint_offset, i00, w[0]);
            sparse_values.emplace_back(constraint_offset, i01, w[1]);
            sparse_values.emplace_back(constraint_offset, i11, w[2]);
            sparse_values.emplace_back(constraint_offset, i10, w[3]);
            b(constraint_offset) = dst_point.x;
            constraint_offset++;

            sparse_values.emplace_back(constraint_offset, i00 + 1, w[0]);
            sparse_values.emplace_back(constraint_offset, i01 + 1, w[1]);
            sparse_values.emplace_back(constraint_offset, i11 + 1, w[2]);
            sparse_values.emplace_back(constraint_offset, i10 + 1, w[3]);
            b(constraint_offset) = dst_point.y;
            constraint_offset++;
        }

        // Add mesh similarity error constraints.
        mesh_grid.for_each([&](const int index, const cv::Point& coord) {
            if(coord.x == mesh_size.width - 1 || coord.y == mesh_size.height - 1)
                return;

            // V00 ---- V10
            //  |  .     |
            //  |     .  |
            // V01 ---- V11
            //
            // Upper Triangle Constraints
            // 0 = x10 + u1(x11 - x10) + v1(y11 - y10) - x00
            // 0 = y10 + u1(y11 - y10) + v1(x10 - x11) - y00
            //
            // Lower Triangle Constraints
            // 0 = x01 + u2(x11 - x01) + v2(y11 - y01) - x00
            // 0 = y01 + u2(y11 - y01) + v2(x01 - x11) - y00

            const int i00 = 2 * index, i10 = i00 + 2;
            const int i01 = 2 * (index + mesh_size.width), i11 = i01 + 2;

            const cv::Point2f V00(global_mesh(i00), global_mesh(i00 + 1));
            const cv::Point2f V01(global_mesh(i01), global_mesh(i01 + 1));
            const cv::Point2f V11(global_mesh(i11), global_mesh(i11 + 1));
            const cv::Point2f V10(global_mesh(i10), global_mesh(i10 + 1));

            const cv::Point2f V1 = V00 - V10, R1 = V11 - V10;
            const cv::Point2f V2 = V00 - V01, R2 = V11 - V01;
            const float l1 = 1.0f/R1.dot(R1), l2 = 1.0f/R2.dot(R2);

            const float weight = m_Settings.local_smoothing;

            const float u1 = (R1.x * V1.x + R1.y * V1.y) * l1 * weight;
            const float v1 = (R1.y * V1.x - R1.x * V1.y) * l1 * weight;
            const float u2 = (R2.x * V2.x + R2.y * V2.y) * l2 * weight;
            const float v2 = (R2.y * V2.x - R2.x * V2.y) * l2 * weight;

            // Upper Triangle
            sparse_values.emplace_back(constraint_offset, i00, -weight);
            sparse_values.emplace_back(constraint_offset, i01, weight - u2);
            sparse_values.emplace_back(constraint_offset, i01 + 1, -v2);
            sparse_values.emplace_back(constraint_offset, i11, u2);
            sparse_values.emplace_back(constraint_offset, i11 + 1, v2);
            constraint_offset++;
            sparse_values.emplace_back(constraint_offset, i00 + 1, -weight);
            sparse_values.emplace_back(constraint_offset, i01, v2);
            sparse_values.emplace_back(constraint_offset, i01 + 1, weight - u2);
            sparse_values.emplace_back(constraint_offset, i11, -v2);
            sparse_values.emplace_back(constraint_offset, i11 + 1, u2);
            constraint_offset++;

            // Lower Triangle
            sparse_values.emplace_back(constraint_offset, i00, -weight);
            sparse_values.emplace_back(constraint_offset, i10, weight - u1);
            sparse_values.emplace_back(constraint_offset, i10 + 1, -v1);
            sparse_values.emplace_back(constraint_offset, i11, u1);
            sparse_values.emplace_back(constraint_offset, i11 + 1, v1);
            constraint_offset++;
            sparse_values.emplace_back(constraint_offset, i00 + 1, -weight);
            sparse_values.emplace_back(constraint_offset, i10, v1);
            sparse_values.emplace_back(constraint_offset, i10 + 1, weight - u1);
            sparse_values.emplace_back(constraint_offset, i11, -v1);
            sparse_values.emplace_back(constraint_offset, i11 + 1, u1);
            constraint_offset++;
        });

        A.setFromTriplets(sparse_values.begin(), sparse_values.end());

        // Solve the system to get the optimal motion mesh.
        Eigen::LeastSquaresConjugateGradient<Eigen::SparseMatrix<float>> solver(A);
        Eigen::VectorXf results = solver.solveWithGuess(b, global_mesh);

        // Convert to WarpMesh
        cv::Mat mesh(m_Settings.motion_resolution, CV_32FC2, results.data());
        mesh_grid.for_each_aligned([&](const int index, const cv::Point2f& aligned_coord){
            auto& vertex = mesh.at<cv::Point2f>(index);
            vertex = (aligned_coord - vertex) / region.size();
        });
        return WarpMesh(mesh, true, true);
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
