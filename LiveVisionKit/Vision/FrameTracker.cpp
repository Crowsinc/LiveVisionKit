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

	constexpr float STABILITY_SMOOTHING_FACTOR = 0.06f;

	constexpr float DYN_MODEL_DISTRIBTION_THRESHOLD_Y = 0.25f;
	constexpr float DYN_MODEL_DISTRIBTION_THRESHOLD_X = 0.5f;

//---------------------------------------------------------------------------------------------------------------------

	FrameTracker::FrameTracker(const MotionModel model, const float estimation_threshold, const GridDetector& detector)
		: m_GridDetector(detector),
		  m_TrackingResolution(detector.resolution()),
		  m_MinMatchThreshold(estimation_threshold * detector.feature_capacity()),
		  m_TrackingStability(0.0f),
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

		return cv::Point2f(
			static_cast<float>(frame.cols) / m_TrackingResolution.width,
			static_cast<float>(frame.rows) / m_TrackingResolution.height
		);
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

			update_tracking_stability(m_MatchedPoints.size(), m_TrackedPoints.size());

			return Homography::FromMatrix(motion);
		}
		else return abort_tracking();
	}

//---------------------------------------------------------------------------------------------------------------------

	Homography FrameTracker::abort_tracking()
	{
		update_tracking_stability(0, 0);
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
		// A good Homography should always be better than a good
		// affine transform. So always pick Homography unless the
		// Homography is likely to produce bad results. 
		
		// If the tracking points are not well distributed, then 
		// its possible that the Homography focuses too much on a 
		// resulting in distortions in the under-represented areas.
		// Hence, we switch to affine in this case to ensure the 
		// frame is fairly treated. This effect is particularly 
		// sensitive in the sky, which often has little to no points.
		const auto distribution_error = m_GridDetector.distribution_error();

		bool prefer_affine = distribution_error.x > DYN_MODEL_DISTRIBTION_THRESHOLD_X
			              || distribution_error.y > DYN_MODEL_DISTRIBTION_THRESHOLD_Y;
	
		return prefer_affine ? MotionModel::AFFINE : MotionModel::HOMOGRAPHY;
	}

//---------------------------------------------------------------------------------------------------------------------

	void FrameTracker::update_tracking_stability(const float inliers, const float samples)
	{
		LVK_ASSERT(samples >= 0.0f);
		LVK_ASSERT(inliers <= samples);

		// NOTE: we use the inlier ratio as a measure of stability. 
		// Assuming the GridDetector produces an even spread of points 
		// across the entire frame, as it is designed to do. Then a low
		// inlier proportion suggests that the frame contains competing
		// non-global motions, which makes the tracking prone to errors
		// and the stability is low. If the inlier ratio is high, then
		// the new points, plus our propagated inliers, continue to 
		// consistently describe the global motion of the frame hence
		// the stability is high. 

		float inlier_proportion = 0;
		if(samples != 0)
			inlier_proportion = inliers / samples;

		m_TrackingStability = exponential_moving_average(
			m_TrackingStability,
			inlier_proportion,
			STABILITY_SMOOTHING_FACTOR
		);
	}
	
//---------------------------------------------------------------------------------------------------------------------

	const float FrameTracker::tracking_stability() const
	{
		return m_TrackingStability;
	}

//---------------------------------------------------------------------------------------------------------------------

	const std::vector<cv::Point2f> FrameTracker::tracking_points() const
	{
		return m_ScaledMatchedPoints;
	}

//---------------------------------------------------------------------------------------------------------------------

}
