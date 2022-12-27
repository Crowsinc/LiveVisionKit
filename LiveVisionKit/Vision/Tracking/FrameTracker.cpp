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

#include "Math/Math.hpp"
#include "Utility/Algorithm.hpp"
#include "Diagnostics/Directives.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	constexpr float METRIC_SMOOTHING_FACTOR = 0.05f;
	constexpr float GOOD_DISTRIBUTION_QUALITY = 0.4f;

//---------------------------------------------------------------------------------------------------------------------

	FrameTracker::FrameTracker(const MotionModel model, const float estimation_threshold, const GridDetector& detector)
		: m_GridDetector(detector),
		  m_TrackingResolution(detector.resolution()),
		  m_MinMatchThreshold(static_cast<uint32_t>(
              estimation_threshold * static_cast<float>(detector.feature_capacity())
          )),
		  m_MotionModel(model),
		  m_PrevFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_NextFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_FirstFrame(true)
	{
		LVK_ASSERT(between(estimation_threshold, 0.0f, 1.0f));

		m_TrackedPoints.reserve(m_GridDetector.feature_capacity());
		m_MatchedPoints.reserve(m_GridDetector.feature_capacity());
		m_ScaledTrackedPoints.reserve(m_GridDetector.feature_capacity());
		m_ScaledMatchedPoints.reserve(m_GridDetector.feature_capacity());
		m_InlierStatus.reserve(m_GridDetector.feature_capacity());
		m_MatchStatus.reserve(m_GridDetector.feature_capacity());

		// Light sharpening kernel
		m_FilterKernel = cv::Mat({3, 3}, {
			0.0f, -0.5f,  0.0f,
		   -0.5f,  3.0f, -0.5f,
			0.0f, -0.5f,  0.0f
		});

		// Use MAGSAC for its threshold robustness.
		m_USACParams.sampler = cv::SAMPLING_UNIFORM;
		m_USACParams.score = cv::SCORE_METHOD_MAGSAC;
		m_USACParams.loMethod = cv::LOCAL_OPTIM_SIGMA;
		m_USACParams.maxIterations = 250;
		m_USACParams.confidence = 0.99;
		m_USACParams.loIterations = 10;
		m_USACParams.loSampleSize = 20;
		m_USACParams.threshold = 4;

		restart();
	}

//---------------------------------------------------------------------------------------------------------------------

	void FrameTracker::restart()
	{
		m_FirstFrame = true;
		m_GridDetector.reset();
	}

//---------------------------------------------------------------------------------------------------------------------

	cv::Point2f FrameTracker::import_next(const cv::UMat& frame)
	{
		LVK_ASSERT(frame.type() == CV_8UC1);

		cv::resize(frame, m_NextFrame, m_TrackingResolution, 0, 0, cv::INTER_AREA);
		cv::filter2D(m_NextFrame, m_NextFrame, m_NextFrame.type(), m_FilterKernel);

		return {
            static_cast<float>(frame.cols) / static_cast<float>(m_TrackingResolution.width),
            static_cast<float>(frame.rows) / static_cast<float>(m_TrackingResolution.height)
        };
	}

//---------------------------------------------------------------------------------------------------------------------

	Homography FrameTracker::track(const cv::UMat& next_frame)
	{
		LVK_ASSERT(!next_frame.empty());
		LVK_ASSERT(next_frame.type() == CV_8UC1);

		prepare_state();

		// Import the frame into the tracker
		std::swap(m_PrevFrame, m_NextFrame);
		const auto scaling = import_next(next_frame);

		if(m_FirstFrame)
		{
			m_FirstFrame = false;
			return abort_tracking();
		}

		// Detect tracking points
		m_GridDetector.detect(m_PrevFrame, m_TrackedPoints);

		if (m_TrackedPoints.size() < m_MinMatchThreshold)
			return abort_tracking();

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

		fast_filter(m_TrackedPoints, m_MatchedPoints, m_MatchStatus);

		if (m_MatchedPoints.size() < m_MinMatchThreshold)
			return abort_tracking();

		// Re-scale all the points to original frame size otherwise the motion will be downscaled
		for(size_t i = 0; i < m_TrackedPoints.size(); i++)
		{
			auto& scaled_tracked_point = m_ScaledTrackedPoints.emplace_back(m_TrackedPoints[i]);
			scaled_tracked_point.x *= scaling.x;
			scaled_tracked_point.y *= scaling.y;

			auto& scaled_matched_point = m_ScaledMatchedPoints.emplace_back(m_MatchedPoints[i]);
			scaled_matched_point.x *= scaling.x;
			scaled_matched_point.y *= scaling.y;
		}

		// Estimate motion between frames
		MotionModel motion_model = m_MotionModel;
		if (motion_model == MotionModel::DYNAMIC)
			motion_model = choose_optimal_model();

		cv::Mat motion;
		switch(motion_model)
		{
			case MotionModel::AFFINE:
				//TODO: switch to USAC when partial is supported
				motion = cv::estimateAffinePartial2D(
					m_ScaledTrackedPoints,
					m_ScaledMatchedPoints,
					m_InlierStatus,
					cv::RANSAC
				);
				break;
			case MotionModel::HOMOGRAPHY:
				motion = cv::findHomography(
					m_ScaledTrackedPoints,
					m_ScaledMatchedPoints,
					m_InlierStatus,
					m_USACParams
				);
				break;
		}

		if (!motion.empty())
		{
			// NOTE: We only propagate inlier points to  the GridDetector to 
			// help ensure consistency between subsequent motion estimations.
			// Additionally, the GridDetector doesn't detect new points if the 
			// propagated points meet the detection load. This means that outliers
			// are naturally removed from the tracking point set until we have lost
			// too many inliers, and the GridDetector has to detect new points. 

			fast_filter(m_MatchedPoints, m_ScaledMatchedPoints, m_InlierStatus);
			m_GridDetector.propagate(m_MatchedPoints);

			update_metrics(
				static_cast<float>(m_MatchedPoints.size()) / static_cast<float>(m_TrackedPoints.size()),
				m_GridDetector.distribution_quality()
			);

			return Homography::FromMatrix(motion);
		}
		else return abort_tracking();
	}

//---------------------------------------------------------------------------------------------------------------------

	Homography FrameTracker::abort_tracking()
	{
		update_metrics(0, m_GridDetector.distribution_quality());

		m_GridDetector.reset();
		return Homography::Identity();
	}

//---------------------------------------------------------------------------------------------------------------------

	void FrameTracker::prepare_state()
	{
		m_ScaledTrackedPoints.clear();
		m_ScaledMatchedPoints.clear();
		m_TrackedPoints.clear();
		m_MatchedPoints.clear();
		m_InlierStatus.clear();
		m_MatchStatus.clear();
	}

//---------------------------------------------------------------------------------------------------------------------

	void FrameTracker::set_model(const MotionModel& model)
	{
		m_MotionModel = model;
	}

//---------------------------------------------------------------------------------------------------------------------

	MotionModel FrameTracker::model() const
	{
		return m_MotionModel;
	}

//---------------------------------------------------------------------------------------------------------------------

	MotionModel FrameTracker::choose_optimal_model() const
	{
		// A good Homography should always be better than a good affine 
		// transform. But if the tracking points are not well distributed,
		// then it's possible that the Homography focuses too much on a 
		// specific area, leading to distortions elsewhere. Hence, we will
		// only consider using a Homography if the tracking point distribution
		// is a fair representation of the frame. Otherwise we use the affine
		// transform, which always fully represents the frame. 

		// TODO: Use a more rigorous approach to selecting the optimal model.
		const auto aspect_ratio = m_GridDetector.resolution().aspectRatio();
		const bool good_distribution = m_DistributionQuality.x >= GOOD_DISTRIBUTION_QUALITY
								    && m_DistributionQuality.y / aspect_ratio >= GOOD_DISTRIBUTION_QUALITY;

		return good_distribution ? MotionModel::HOMOGRAPHY : MotionModel::AFFINE;
	}

//---------------------------------------------------------------------------------------------------------------------

	void FrameTracker::update_metrics(const float inlier_ratio, const cv::Point2f distribution_quality)
	{
		LVK_ASSERT(between(inlier_ratio, 0.0f, 1.0f));
		LVK_ASSERT(between(distribution_quality.x, 0.0f, 1.0f));
		LVK_ASSERT(between(distribution_quality.y, 0.0f, 1.0f));

		m_DistributionQuality = exp_moving_average(
            m_DistributionQuality, distribution_quality, METRIC_SMOOTHING_FACTOR
        );

		m_SceneStability = exp_moving_average(
            m_SceneStability, inlier_ratio, METRIC_SMOOTHING_FACTOR
        );
	}
	
//---------------------------------------------------------------------------------------------------------------------

	float FrameTracker::stability() const
	{
		return m_SceneStability;
	}

//---------------------------------------------------------------------------------------------------------------------

	const std::vector<cv::Point2f>& FrameTracker::tracking_points() const
	{
		return m_ScaledMatchedPoints;
	}

//---------------------------------------------------------------------------------------------------------------------

}
