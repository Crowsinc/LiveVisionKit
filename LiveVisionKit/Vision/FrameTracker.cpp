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
#include <util/platform.h>

namespace lvk
{
//---------------------------------------------------------------------------------------------------------------------

	constexpr double MAX_TRACKING_ERROR = 15;

//---------------------------------------------------------------------------------------------------------------------

	FrameTracker::FrameTracker(const MotionModel model, const float estimation_threshold, const GridDetector& detector)
		: m_GridDetector(detector),
		  m_TrackingResolution(detector.resolution()),
		  m_MinMatchThreshold(estimation_threshold * detector.detection_target()),
		  m_MotionModel(model),
		  m_PrevFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_NextFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_FirstFrame(true)
	{
		LVK_ASSERT(between(estimation_threshold, 0.0f, 1.0f));

		m_TrackedPoints.reserve(m_GridDetector.detection_target());
		m_MatchedPoints.reserve(m_GridDetector.detection_target());
		m_ScaledTrackedPoints.reserve(m_GridDetector.detection_target());
		m_ScaledMatchedPoints.reserve(m_GridDetector.detection_target());
		m_TrackingError.reserve(m_GridDetector.detection_target());
		m_InlierStatus.reserve(m_GridDetector.detection_target());
		m_MatchStatus.reserve(m_GridDetector.detection_target());

		// Use light sharpening kernel
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
		m_TrackedPoints.clear();
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

		std::swap(m_PrevFrame, m_NextFrame);
		const auto scaling = import_next(next_frame);

		if(m_FirstFrame)
		{
			m_FirstFrame = false;
			return Homography::Identity();
		}

		// Detect tracking points
		m_GridDetector.detect(m_PrevFrame, m_TrackedPoints);

		if(m_TrackedPoints.size() < m_MinMatchThreshold)
			return Homography::Identity();

		// Match tracking points
		cv::calcOpticalFlowPyrLK(
			m_PrevFrame,
			m_NextFrame,
			m_TrackedPoints,
			m_MatchedPoints,
			m_MatchStatus,
			m_TrackingError,
			cv::Size(7, 7)
		);

		for(uint32_t i = 0; i < m_MatchStatus.size(); i++)
			m_MatchStatus[i] = m_MatchStatus[i] && (m_TrackingError[i] < MAX_TRACKING_ERROR);

		fast_filter(m_TrackedPoints, m_MatchedPoints, m_MatchStatus);

		if(m_MatchedPoints.size() < m_MinMatchThreshold)
			return Homography::Identity();

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

		cv::Mat motion;
		switch(m_MotionModel)
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

		m_GridDetector.reset();

		if(!motion.empty())
		{
			// NOTE: Propogate inlier matches on to the next detection.
			// This means that we are using the same points for consecutive
			// motion estimation which aids in consistency. It also allows
			// the GridDetector to skip detection of new points if the
			// propogated points alone meet the detection load. As we only
			// propogate inliers, it also means that outliers are naturally
			// filtered out until the detection load is too low.

			fast_filter(m_MatchedPoints, m_InlierStatus);
			m_GridDetector.propogate(m_MatchedPoints);

			return Homography::FromMatrix(motion);
		}
		else return Homography::Identity();
	}

//---------------------------------------------------------------------------------------------------------------------

	void FrameTracker::prepare_state()
	{
		m_ScaledTrackedPoints.clear();
		m_ScaledMatchedPoints.clear();
		m_TrackedPoints.clear();
		m_MatchedPoints.clear();
		m_TrackingError.clear();
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

	const std::vector<cv::Point2f> FrameTracker::tracking_points() const
	{
		return m_ScaledMatchedPoints;
	}

//---------------------------------------------------------------------------------------------------------------------

}
