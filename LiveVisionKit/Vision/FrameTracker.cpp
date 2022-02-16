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

#include "../Math/Math.hpp"
#include "../Utility/Algorithm.hpp"
#include "../Diagnostics/Assert.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	static constexpr double DEFAULT_FEATURE_THRESHOLD = 70;
	static constexpr double MAX_FEATURE_THRESHOLD = 250;
	static constexpr double MIN_FEATURE_THRESHOLD = 10;

	static constexpr double MAX_TRACKING_ERROR = 15;

	static constexpr uint32_t REGION_ROWS = 1;
	static constexpr uint32_t REGION_COLUMNS = 3;
	static constexpr uint32_t REGION_DETECTION_TARGET = 1000;

//---------------------------------------------------------------------------------------------------------------------

	FrameTracker::FrameTracker(const float estimation_threshold, const cv::Size& resolution, const cv::Size& block_size)
		: m_TrackingGrid(resolution, block_size),
		  m_TrackingPointTarget(m_TrackingGrid.block_count()),
		  m_MinMatchThreshold(estimation_threshold * m_TrackingPointTarget),
		  m_TrackingResolution(resolution),
		  m_PrevFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_NextFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_FirstFrame(true)
	{
		LVK_ASSERT(between(estimation_threshold, 0.0f, 1.001f));

		m_Features.reserve(5000);

		m_TrackedPoints.reserve(m_TrackingPointTarget);
		m_MatchedPoints.reserve(m_TrackingPointTarget);
		m_ScaledTrackedPoints.reserve(m_TrackingPointTarget);
		m_ScaledMatchedPoints.reserve(m_TrackingPointTarget);

		m_MatchStatus.reserve(m_TrackingPointTarget);
		m_InlierStatus.reserve(m_TrackingPointTarget);
		m_TrackingError.reserve(m_TrackingPointTarget);

		initialise_regions(
			REGION_ROWS,
			REGION_COLUMNS,
			REGION_DETECTION_TARGET
		);

		restart();
	}

//---------------------------------------------------------------------------------------------------------------------

	void FrameTracker::initialise_regions(const uint32_t rows, const uint32_t cols, const uint32_t detection_target)
	{
		// NOTE: We divide the frame across multiple tracking regions to more evenly
		// distribute feature detection across the frame. Using regions also accounts
		// for differing scenery across the frame by allowing them to dynamically
		// adjust their threshold to meet a feature target.

		m_TrackingRegions.clear();

		const cv::Size region_size(
			m_TrackingResolution.width / cols,
			m_TrackingResolution.height / rows
		);

		for(uint32_t r = 0; r < rows; r++)
		{
			for(uint32_t c = 0; c < cols; c++)
			{
				auto& region = m_TrackingRegions.emplace_back();
				region.region = cv::Rect(
					cv::Point(c * region_size.width, r * region_size.height),
					region_size
				);
				region.detection_target = detection_target;
			}
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void FrameTracker::restart()
	{
		m_FirstFrame = true;
		m_TrackedPoints.clear();
		m_TrackingGrid.reset();

		for(auto& [region, feature_threshold, feature_target] : m_TrackingRegions)
			feature_threshold = DEFAULT_FEATURE_THRESHOLD;
	}

//---------------------------------------------------------------------------------------------------------------------

	cv::Point2f FrameTracker::import_next(const cv::UMat& frame)
	{
		LVK_ASSERT(frame.type() == CV_8UC1);

		cv::resize(frame, m_NextFrame, m_TrackingResolution, 0, 0, cv::INTER_AREA);

		const cv::Mat sharpening_kernel({3,3}, {
				 0.0f, -0.5f, 0.0f,
				-0.5f, 3.0f, -0.5f,
				 0.0f, -0.5f, 0.0f
		});

		cv::filter2D(m_NextFrame, m_NextFrame, m_NextFrame.type(), sharpening_kernel);

		return cv::Point2f(
			static_cast<float>(frame.cols) / m_TrackingResolution.width,
			static_cast<float>(frame.rows) / m_TrackingResolution.height
		);
	}

//---------------------------------------------------------------------------------------------------------------------

	Homography FrameTracker::track(const cv::UMat& next_frame)
	{
		LVK_ASSERT(!next_frame.empty() && next_frame.type() == CV_8UC1);

		prepare_state();

		std::swap(m_PrevFrame, m_NextFrame);
		const auto scaling = import_next(next_frame);

		if(m_FirstFrame)
		{
			m_FirstFrame = false;
			return Homography::Identity();
		}

		// Feature detection

		for(auto& [region, feature_threshold, detection_target] : m_TrackingRegions)
		{
			m_Features.clear();

			cv::FAST(
				m_PrevFrame(region),
				m_Features,
				feature_threshold,
				true
			);

			m_TrackingGrid.process(m_Features, {1, 1}, region.tl());

			// Dynamically adjust feature threshold to try meet detection target next time
			if(m_Features.size() > detection_target)
				feature_threshold = lerp(feature_threshold, MAX_FEATURE_THRESHOLD, 0.1);
			else
				feature_threshold = lerp(feature_threshold, MIN_FEATURE_THRESHOLD, 0.1);
		}

		// Add new tracking points to meet the tracking point target
		const int points_to_add = std::max(m_TrackingPointTarget - static_cast<int>(m_TrackedPoints.size()), 0);
		m_TrackingGrid.extract(m_TrackedPoints, points_to_add);

		if(m_TrackedPoints.size() < m_MinMatchThreshold)
			return Homography::Identity();

		// Feature matching

		cv::calcOpticalFlowPyrLK(
			m_PrevFrame,
			m_NextFrame,
			m_TrackedPoints,
			m_MatchedPoints,
			m_MatchStatus,
			m_TrackingError
		);

		for(uint32_t i = 0; i < m_MatchStatus.size(); i++)
			m_MatchStatus[i] = m_MatchStatus[i] && m_TrackingError[i] < MAX_TRACKING_ERROR;

		fast_filter(m_TrackedPoints, m_MatchedPoints, m_MatchStatus);

		if(m_MatchedPoints.size() < m_MinMatchThreshold)
			return Homography::Identity();

		// Motion Estimation

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

		const auto affine_estimate = cv::estimateAffinePartial2D(
			m_ScaledTrackedPoints,
			m_ScaledMatchedPoints,
			m_InlierStatus,
			cv::RANSAC
		);

		// Propogate good matched points to next pass
		fast_filter(m_MatchedPoints, m_InlierStatus);
		m_TrackedPoints = m_MatchedPoints;

		m_TrackingGrid.reset_mask(true);
		m_TrackingGrid.mask(m_TrackedPoints, false);

		return affine_estimate.empty() ? Homography::Identity() : Homography::FromAffineMatrix(affine_estimate);
	}

//---------------------------------------------------------------------------------------------------------------------

	void FrameTracker::prepare_state()
	{
		m_ScaledTrackedPoints.clear();
		m_ScaledMatchedPoints.clear();
		m_MatchedPoints.clear();
		m_TrackingError.clear();
		m_InlierStatus.clear();
		m_MatchStatus.clear();

		m_TrackingGrid.reset_grid();
	}

//---------------------------------------------------------------------------------------------------------------------

	const std::vector<cv::Point2f> FrameTracker::tracking_points() const
	{
		return m_ScaledMatchedPoints;
	}

//---------------------------------------------------------------------------------------------------------------------

}
