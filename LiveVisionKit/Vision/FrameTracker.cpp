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

//---------------------------------------------------------------------------------------------------------------------

	FrameTracker::FrameTracker(const FrameTrackerSettings& settings)
        : m_OpticalTracker(cv::SparsePyrLKOpticalFlow::create(
              OPTICAL_TRACKER_WIN_SIZE, OPTICAL_TRACKER_PYR_LEVELS,
              cv::TermCriteria(
                  cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                  OPTICAL_TRACKER_MAX_ITERS, 0.01
              )
          ))
	{
        configure(settings);

		restart();
	}

//---------------------------------------------------------------------------------------------------------------------

    void FrameTracker::configure(const FrameTrackerSettings& settings)
    {
        LVK_ASSERT(settings.motion_resolution.height >= WarpMesh::MinimumSize.height);
        LVK_ASSERT(settings.motion_resolution.width >= WarpMesh::MinimumSize.width);
        LVK_ASSERT(settings.acceptance_threshold >= 0.0f);
        LVK_ASSERT(settings.temporal_smoothing >= 0.0f);
        LVK_ASSERT(settings.local_smoothing >= 0.0f);
        LVK_ASSERT(settings.min_motion_samples >= 4);
        LVK_ASSERT_01(settings.stability_threshold);
        LVK_ASSERT_01(settings.uniformity_threshold);

        m_FeatureDetector.configure(settings);
        m_MatchStatus.reserve(m_FeatureDetector.max_feature_capacity());
        m_InlierStatus.reserve(m_FeatureDetector.max_feature_capacity());
        m_TrackedPoints.reserve(m_FeatureDetector.max_feature_capacity());
        m_MatchedPoints.reserve(m_FeatureDetector.max_feature_capacity());
        m_TrackingRegion = cv::Rect2f({0,0}, settings.detection_resolution);

        if(settings.motion_resolution != m_Settings.motion_resolution || m_MeshConstraints.empty())
        {
            m_OptimizedMesh = Eigen::VectorXf::Zero(2 * settings.motion_resolution.area());
            m_StaticConstraintCount = generate_mesh_constraints(
                m_TrackingRegion,
                settings.motion_resolution,
                m_MeshConstraints
            );
        }

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
        m_TrackedFeatures.clear();
        m_FeatureDetector.reset();
        m_FrameInitialized = false;
        m_OptimizedMesh = Eigen::VectorXf::Zero(m_Settings.motion_resolution.area() * 2);
	}

//---------------------------------------------------------------------------------------------------------------------

    std::optional<WarpMesh> FrameTracker::track(const cv::UMat& next_frame)
	{
		LVK_ASSERT(!next_frame.empty() && next_frame.type() == CV_8UC1);

        // Reset tracking metrics
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

        // Detect features in the current frame.
        const auto distribution = m_FeatureDetector.detect(m_CurrentFrame, m_TrackedFeatures);
        if(m_TrackedFeatures.size() < m_Settings.min_motion_samples || distribution < m_Settings.uniformity_threshold)
        {
            m_TrackedFeatures.clear();
            return std::nullopt;
        }

        // Convert features to tracking points
        m_TrackedPoints.clear();
        for(const auto& feature : m_TrackedFeatures)
            m_TrackedPoints.emplace_back(feature.pt);

		// Match tracking points.
        m_OpticalTracker->calc(
            m_PreviousFrame,
            m_CurrentFrame,
            m_TrackedPoints,
            m_MatchedPoints,
            m_MatchStatus
        );

        // Filter out unmatched points
        fast_filter(m_TrackedFeatures, m_TrackedPoints, m_MatchedPoints, m_MatchStatus);
        if(m_MatchedPoints.size() < m_Settings.min_motion_samples)
        {
            m_TrackedFeatures.clear();
            return std::nullopt;
        }

        // Estimate motion using the tracking results
        WarpMesh motion(m_Settings.motion_resolution);
        if(m_Settings.track_local_motions)
        {
            estimate_local_motions(
                motion,
                m_TrackingRegion,
                m_TrackedPoints, m_MatchedPoints,
                m_InlierStatus
            );
        }
        else
        {
            estimate_global_motion(
                motion,
                distribution > HOMOGRAPHY_DISTRIBUTION_THRESHOLD,
                m_TrackingRegion,
                m_TrackedPoints, m_MatchedPoints,
                m_InlierStatus
            );
        }

        // Drop the frame if the inlier ratio is below the expected threshold.
        // This usually happens due to an un-trackable frame or discontinuity.
        const auto inlier_ratio = ratio_of<uint8_t>(m_InlierStatus, 1);
        if(inlier_ratio < m_Settings.stability_threshold)
        {
            m_TrackedFeatures.clear();
            return std::nullopt;
        }

        // Filter outliers so we're left with only high quality points,
        // then propagate them so that they're re-used in the detector.
        for(int i = m_InlierStatus.size() - 1; i >= 0; i--)
        {
            if(m_InlierStatus[i])
            {
                // Borrow class id integer to store feature age.
                m_TrackedFeatures[i].class_id++;
                m_TrackedFeatures[i].pt = m_MatchedPoints[i];
            }
            else fast_erase(m_TrackedFeatures, i);
        }
        m_FeatureDetector.propagate(m_TrackedFeatures);

        return std::move(motion);
	}

//---------------------------------------------------------------------------------------------------------------------

    void FrameTracker::estimate_local_motions(
        WarpMesh& motion_mesh,
        const cv::Rect2f& region,
        const std::vector<cv::Point2f>& tracked_points,
        const std::vector<cv::Point2f>& matched_points,
        std::vector<uint8_t>& inlier_status
    )
    {
        LVK_ASSERT(tracked_points.size() == matched_points.size());

        const auto mesh_size = motion_mesh.size();
        const auto grid_size = mesh_size - cv::Size(1, 1);

        // Create a partitioned grid for the mesh points.
        const VirtualGrid mesh_grid(mesh_size, cv::Rect2f(
            region.tl(), (cv::Size2f(mesh_size) / cv::Size2f(grid_size)) * region.size()
        ));

        // Initialize linear system to optimize the mesh
        const int constraints = m_StaticConstraintCount + 2 * tracked_points.size();
        Eigen::SparseMatrix<float> A(constraints, 2 * mesh_size.area());
        Eigen::VectorXf b = Eigen::VectorXf::Zero(A.rows());

        // Finalize temporal smoothing constraints with the previous optimized mesh.
        int constraint_offset = 0;
        mesh_grid.for_each([&](const int index, const cv::Point2f& coord){
            const int x_index = 2 * index, y_index = x_index + 1;
            b(constraint_offset++) = m_Settings.temporal_smoothing * m_OptimizedMesh(x_index);
            b(constraint_offset++) = m_Settings.temporal_smoothing * m_OptimizedMesh(y_index);
        });

        // Jump to the end of the static mesh constraints to add dynamic ones
        const int static_triplet_count = m_MeshConstraints.size();
        constraint_offset = m_StaticConstraintCount;

        // Add feature warping constraints
        for(size_t i = 0; i < tracked_points.size(); i++)
        {
            const cv::Point2f& src_point = tracked_points[i];
            const cv::Point2f& dst_point = matched_points[i];

            // Resolve the mesh vertices surrounding the feature.
            cv::Point k00 = mesh_grid.key_of(src_point);
            k00.x = std::clamp(k00.x, 0, grid_size.width);
            k00.y = std::clamp(k00.y, 0, grid_size.height);
            const cv::Point k11 = k00 + 1;

            // Get indices of the mesh vertices
            const int i00 = 2 * static_cast<int>(mesh_grid.key_to_index(k00));
            const int i11 = 2 * static_cast<int>(mesh_grid.key_to_index(k11));
            const int i10 = i00 + 2, i01 = i11 - 2;

            // Get barycentric weights of the src point in its cell,
            // we want these weights to hold in the final motion mesh.
            const cv::Scalar w = barycentric_rect(
                {mesh_grid.key_to_point(k00), mesh_grid.key_to_point(k11)}, src_point
            );

            m_MeshConstraints.emplace_back(constraint_offset, i00, w[0]);
            m_MeshConstraints.emplace_back(constraint_offset, i01, w[1]);
            m_MeshConstraints.emplace_back(constraint_offset, i11, w[2]);
            m_MeshConstraints.emplace_back(constraint_offset, i10, w[3]);
            b(constraint_offset) = dst_point.x;
            constraint_offset++;

            m_MeshConstraints.emplace_back(constraint_offset, i00 + 1, w[0]);
            m_MeshConstraints.emplace_back(constraint_offset, i01 + 1, w[1]);
            m_MeshConstraints.emplace_back(constraint_offset, i11 + 1, w[2]);
            m_MeshConstraints.emplace_back(constraint_offset, i10 + 1, w[3]);
            b(constraint_offset) = dst_point.y;
            constraint_offset++;
        }

        // Solve the system to get the optimal motion mesh.
        A.setFromTriplets(m_MeshConstraints.begin(), m_MeshConstraints.end());
        Eigen::LeastSquaresConjugateGradient<Eigen::SparseMatrix<float>> solver(A);
        m_OptimizedMesh = solver.solveWithGuess(b, m_OptimizedMesh);

        // Update inlier status of all points
        inlier_status.resize(tracked_points.size());
        for(int i = 0; i < tracked_points.size(); i++)
        {
            const int quad_x_index = i * 8 + static_triplet_count;
            const int quad_y_index = quad_x_index + 4;


            const auto& qx0 = m_MeshConstraints[quad_x_index + 0];
            const auto& qx1 = m_MeshConstraints[quad_x_index + 1];
            const auto& qx2 = m_MeshConstraints[quad_x_index + 2];
            const auto& qx3 = m_MeshConstraints[quad_x_index + 3];

            const float x = qx0.value() * m_OptimizedMesh(qx0.col())
                          + qx1.value() * m_OptimizedMesh(qx1.col())
                          + qx2.value() * m_OptimizedMesh(qx2.col())
                          + qx3.value() * m_OptimizedMesh(qx3.col());


            const auto& qy0 = m_MeshConstraints[quad_y_index + 0];
            const auto& qy1 = m_MeshConstraints[quad_y_index + 1];
            const auto& qy2 = m_MeshConstraints[quad_y_index + 2];
            const auto& qy3 = m_MeshConstraints[quad_y_index + 3];

            const float y = qy0.value() * m_OptimizedMesh(qy0.col())
                          + qy1.value() * m_OptimizedMesh(qy1.col())
                          + qy2.value() * m_OptimizedMesh(qy2.col())
                          + qy3.value() * m_OptimizedMesh(qy3.col());

            const int x_constraint = i * 2 + m_StaticConstraintCount, y_constraint = x_constraint + 1;
            const auto error = std::abs(x - b(x_constraint)) + std::abs(y - b(y_constraint));
            inlier_status[i] = error < m_Settings.acceptance_threshold;
        }

        // Reset mesh constraints back to the static ones.
        m_MeshConstraints.resize(static_triplet_count);

        // Upload results into the motion mesh as offsets
        auto& mesh_offsets = motion_mesh.offsets();
        const cv::Mat mesh(mesh_offsets.size(), CV_32FC2, m_OptimizedMesh.data());
        mesh_grid.for_each_aligned([&](const int index, const cv::Point2f& aligned_coord){
            mesh_offsets.at<cv::Point2f>(index) = (aligned_coord - mesh.at<cv::Point2f>(index)) / region.size();
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    void FrameTracker::estimate_global_motion(
            WarpMesh& motion_mesh,
            const bool homography,
            const cv::Rect2f& region,
            const std::vector<cv::Point2f>& tracked_points,
            const std::vector<cv::Point2f>& matched_points,
            std::vector<uint8_t>& inlier_status
    )
    {
        LVK_ASSERT(tracked_points.size() == matched_points.size());
        LVK_ASSERT(tracked_points.size() >= 4);

        cv::UsacParams params;
        params.threshold = m_Settings.acceptance_threshold;
        params.confidence = 0.99;
        params.maxIterations = 50;
        params.sampler = cv::SAMPLING_UNIFORM;
        params.score = cv::SCORE_METHOD_MAGSAC;
        params.loMethod = cv::LOCAL_OPTIM_SIGMA;
        params.loIterations = 10;
        params.loSampleSize = 20;
        params.final_polisher = cv::MAGSAC;
        params.final_polisher_iterations = 5;

        if(homography)
        {
            motion_mesh.set_to(
                Homography(cv::findHomography(
                    tracked_points,
                    matched_points,
                    inlier_status,
                    params
                )),
                region.size()
            );
        }
        else
        {
            motion_mesh.set_to(
                Homography::FromAffineMatrix(cv::estimateAffinePartial2D(
                    tracked_points,
                    matched_points,
                    inlier_status,
                    cv::RANSAC,
                    params.threshold,
                    params.maxIterations
                )),
                region.size()
            );
        }
    }


//---------------------------------------------------------------------------------------------------------------------

    int FrameTracker::generate_mesh_constraints(
        const cv::Rect2f& region,
        const cv::Size& mesh_size,
        std::vector<Eigen::Triplet<float>>& constraints
    )
    {
        // Create the partitioned grid for the mesh.
        const auto grid_size = mesh_size - cv::Size(1, 1);
        const VirtualGrid mesh_grid(mesh_size, cv::Rect2f(
            region.tl(), (cv::Size2f(mesh_size) / cv::Size2f(grid_size)) * region.size()
        ));

        // Clear past constraints
        constraints.clear();
        int constraint_offset = 0;

        // Add temporal smoothness constraints to the mesh.
        // NOTE: accompanying B vector must be set to past mesh vertices.
        mesh_grid.for_each([&](const int index, const cv::Point2f& coord){
            const int x_index = 2 * index, y_index = x_index + 1;
            constraints.emplace_back(constraint_offset++, x_index,  m_Settings.temporal_smoothing);
            constraints.emplace_back(constraint_offset++, y_index,  m_Settings.temporal_smoothing);
        });

        // Add local mesh smoothness constraints
        // NOTE: accompanying B vector must be set to zero.
        const auto v1 = -mesh_grid.key_size().aspectRatio(), v2 = -1.0 / v1;
        mesh_grid.for_each([&](const int index, const cv::Point& coord) {
            // Minimize some of the optimization load by only applying the constraint
            // where needed. In particular, all edge quads and then a checkerboard
            // pattern. To help with global consistency, larger quads are also added.
            int quad_size = 1;
            if(coord.x % 4 == 0 && coord.y % 4 == 0)
                quad_size = 3;
            else if((coord.x + coord.y) % 2 != 1
                  && coord.x != 0 && coord.y != 0
                  && coord.x != mesh_size.width - 2
                  && coord.y != mesh_size.height - 2
            ) return;

            // Ensure we don't go out of bounds.
            if(coord.x >= mesh_size.width - quad_size || coord.y >= mesh_size.height - quad_size)
                return;

            // Grab mesh vertex indices
            const int i00 = 2 * index, i10 = i00 + 2 * quad_size;
            const int i01 = 2 * (index + quad_size * mesh_size.width), i11 = i01 + 2 * quad_size;

            const float weight = m_Settings.local_smoothing;
            const float w1 = v1 * weight, w2 = v2 * weight;

            // Upper Triangle
            constraints.emplace_back(constraint_offset, i00, -weight);
            constraints.emplace_back(constraint_offset, i01, weight);
            constraints.emplace_back(constraint_offset, i01 + 1, -w2);
            constraints.emplace_back(constraint_offset, i11 + 1, w2);
            constraint_offset++;
            constraints.emplace_back(constraint_offset, i00 + 1, -weight);
            constraints.emplace_back(constraint_offset, i01, w2);
            constraints.emplace_back(constraint_offset, i01 + 1, weight);
            constraints.emplace_back(constraint_offset, i11, -w2);
            constraint_offset++;

            // Lower Triangle
            constraints.emplace_back(constraint_offset, i00, -weight);
            constraints.emplace_back(constraint_offset, i10, weight);
            constraints.emplace_back(constraint_offset, i10 + 1, -w1);
            constraints.emplace_back(constraint_offset, i11 + 1, w1);
            constraint_offset++;
            constraints.emplace_back(constraint_offset, i00 + 1, -weight);
            constraints.emplace_back(constraint_offset, i10, w1);
            constraints.emplace_back(constraint_offset, i10 + 1, weight);
            constraints.emplace_back(constraint_offset, i11, -w1);
            constraint_offset++;
        });

        return constraint_offset;
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

	const std::vector<cv::KeyPoint>& FrameTracker::features() const
	{
		return m_TrackedFeatures;
	}

//---------------------------------------------------------------------------------------------------------------------

    void FrameTracker::draw_trackers(cv::UMat& dst, const cv::Scalar& colour, const int size, const int thickness) const
    {
        LVK_ASSERT(thickness > 0);
        LVK_ASSERT(size > 0);

        std::vector<cv::Point2f> tracking_points;
        tracking_points.reserve(m_TrackedFeatures.size());
        cv::KeyPoint::convert(m_TrackedFeatures, tracking_points);

        draw_crosses(
            dst,
            tracking_points,
            colour,
            size, 4,
            cv::Size2f(dst.size()) / cv::Size2f(m_Settings.detection_resolution)
        );
    }

//---------------------------------------------------------------------------------------------------------------------

}
