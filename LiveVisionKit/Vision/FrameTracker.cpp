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
	static constexpr double MIN_FEATURE_THRESHOLD = 20;

	static constexpr double MAX_TRACKING_ERROR = 20;

	static constexpr uint32_t REGION_ROWS = 1;
	static constexpr uint32_t REGION_COLUMNS = 3;
	static constexpr uint32_t REGION_FEATURE_TARGET = 2000;

//---------------------------------------------------------------------------------------------------------------------

	FrameTracker::FrameTracker(const float estimation_threshold, const cv::Size& resolution, const cv::Size& block_size)
		: m_TrackingResolution(resolution),
		  m_BlockSize(block_size),
		  m_GridSize(
			  std::ceil(static_cast<float>(resolution.width)/block_size.width),
			  std::ceil(static_cast<float>(resolution.height)/block_size.height)
		  ),
		  m_MinMatchThreshold(estimation_threshold * m_GridSize.area()),
		  m_PrevFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_NextFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_FirstFrame(true)
	{
		LVK_ASSERT(estimation_threshold >= 0 && estimation_threshold <= 1.0);
		LVK_ASSERT(resolution.width > 0 && resolution.height > 0);
		LVK_ASSERT(block_size.width > 0 && block_size.width <= resolution.width);
		LVK_ASSERT(block_size.height > 0 && block_size.height <= resolution.height);

		m_Features.reserve(5000);
		m_TrackedPoints.reserve(m_GridSize.area());
		m_MatchedPoints.reserve(m_GridSize.area());
		m_MatchStatus.reserve(m_GridSize.area());
		m_TrackingError.reserve(m_GridSize.area());
		m_InlierStatus.reserve(m_GridSize.area());

		m_Grid.resize(m_GridSize.area());
		m_GridMask.resize(m_GridSize.area(), true);

		initialise_regions(
			REGION_ROWS,
			REGION_COLUMNS,
			REGION_FEATURE_TARGET
		);

		restart();
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

	Transform FrameTracker::track(const cv::UMat& next_frame)
	{
		LVK_ASSERT(!next_frame.empty() && next_frame.type() == CV_8UC1);

		m_TrackedPoints.clear();
		m_MatchedPoints.clear();
		m_TrackingError.clear();
		m_InlierStatus.clear();
		m_MatchStatus.clear();

		std::swap(m_PrevFrame, m_NextFrame);
		const auto scaling = import_next(next_frame);

		if(m_FirstFrame)
		{
			m_FirstFrame = false;
			return Transform::Identity();
		}

		// Feature detection
		for(auto& [region, feature_threshold, feature_target] : m_TrackingRegions)
		{
			m_Features.clear();

			cv::FAST(
				m_PrevFrame(region),
				m_Features,
				feature_threshold,
				true
			);

			process_features(m_Features, m_TrackedPoints, region.tl());

			// Dynamically adjust feature threshold to meet target
			if(m_Features.size() > feature_target)
				feature_threshold = lerp(feature_threshold, MAX_FEATURE_THRESHOLD, 0.1);
			else
				feature_threshold = lerp(feature_threshold, MIN_FEATURE_THRESHOLD, 0.1);
		}

		if(m_TrackedPoints.size() < m_MinMatchThreshold)
			return Transform::Identity();

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
			return Transform::Identity();

		// Motion Estimation

		// Re-scale all the points to original frame size otherwise the motion will be downscaled
		for(size_t i = 0; i < m_TrackedPoints.size(); i++)
		{
			auto& tracked_point = m_TrackedPoints[i];
			tracked_point.x *= scaling.x;
			tracked_point.y *= scaling.y;

			auto& matched_point = m_MatchedPoints[i];
			matched_point.x *= scaling.x;
			matched_point.y *= scaling.y;
		}

		const auto affine_estimate = cv::estimateAffinePartial2D(m_TrackedPoints, m_MatchedPoints, m_InlierStatus);

		// Block outliers of the next frame in the grid mask to avoid re-tracking them next pass.
		fast_filter(m_MatchedPoints, m_InlierStatus, true);
		update_grid_mask(m_MatchedPoints, scaling);

		return affine_estimate.empty() ? Transform::Identity() : Transform::FromAffine2D(affine_estimate);
	}

//---------------------------------------------------------------------------------------------------------------------

	void FrameTracker::process_features(
			const std::vector<cv::KeyPoint>& features,
			std::vector<cv::Point2f>& points,
			const cv::Point2f& offset
	)
	{
		// Process all the features into the grid, keeping only the best for each block
		// This enforces better tracking point distribution across the frame, hopefully
		// leading to a better estimation of global motion. Skip blocks which aren't
		// asserted in the grid mask.
		for(auto& feature : features)
		{
			const auto point = feature.pt + offset;
			const uint32_t grid_x = point.x / m_BlockSize.width;
			const uint32_t grid_y = point.y / m_BlockSize.height;
			const uint32_t index = grid_y * m_GridSize.width + grid_x;

			std::optional<cv::KeyPoint>& curr_feature = m_Grid[index];
			if(m_GridMask[index] && (!curr_feature.has_value() || curr_feature->response < feature.response))
			{
				curr_feature = feature;
				curr_feature->pt = point;
			}
		}

		// Push all resulting best features to the points vector
		for(auto& feature : m_Grid)
		{
			if(feature.has_value())
			{
				points.push_back(feature->pt);
				feature.reset();
			}
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void FrameTracker::update_grid_mask(const std::vector<cv::Point2f>& outliers, const cv::Point2f& scaling)
	{
		// Update the grid mask to disable blocks which contained outliers.

		for(uint32_t i = 0; i < m_GridMask.size(); i++)
			m_GridMask[i] = true;

		const uint32_t scaled_block_width = m_BlockSize.width * scaling.x;
		const uint32_t scaled_block_height = m_BlockSize.height * scaling.y;

		for(auto& outlier : outliers)
		{
			const uint32_t grid_x = outlier.x / scaled_block_width;
			const uint32_t grid_y = outlier.y / scaled_block_height;
			const uint32_t index = grid_y * m_GridSize.width + grid_x;

			m_GridMask[index] = false;
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void FrameTracker::initialise_regions(const uint32_t rows, const uint32_t cols, const uint32_t feature_target)
	{
		// NOTE: We divide the frame across multiple tracking regions to more evenly
		// distribute feature detection across the frame. Using regions alsoaccounts
		// for differing scenery across the frame, by allowing them to dynamically
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
				region.feature_target = feature_target;
			}
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void FrameTracker::restart()
	{
		m_FirstFrame = true;

		for(auto& [region, feature_threshold, feature_target] : m_TrackingRegions)
			feature_threshold = DEFAULT_FEATURE_THRESHOLD;

		for(uint32_t i = 0; i < m_GridMask.size(); i++)
			m_GridMask[i] = true;
	}

//---------------------------------------------------------------------------------------------------------------------

}
