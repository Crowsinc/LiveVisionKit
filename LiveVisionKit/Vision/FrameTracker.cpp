#include "FrameTracker.hpp"

#include <algorithm>
#include <util/platform.h>

#include "../Math/Math.hpp"
#include "../Utility/Algorithm.hpp"
#include "../Diagnostics/Assert.hpp"

namespace lvk
{

	//-------------------------------------------------------------------------------------

	static constexpr double DEFAULT_FEATURE_THRESHOLD = 70;
	static constexpr double MAX_FEATURE_THRESHOLD = 250;
	static constexpr double MIN_FEATURE_THRESHOLD = 30;

	//-------------------------------------------------------------------------------------

	FrameTracker::FrameTracker(const float estimation_threshold, const cv::Size resolution, const cv::Size block_size)
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
		m_Grid.resize(m_GridSize.area());

		// Split the internal resolution in vertical thirds to
		// more evenly distribute points across the frames.
		auto& left_region = m_TrackingRegions.emplace_back();
		left_region.region = cv::Rect(
			cv::Point(0, 0),
			cv::Size(resolution.width/3, resolution.height)
		);
		left_region.feature_target = std::min(4 * m_GridSize.area(), 3000);

		auto& middle_region = m_TrackingRegions.emplace_back();
		middle_region.region = cv::Rect(
			cv::Point(resolution.width/3, 0),
			cv::Size(resolution.width/3, resolution.height)
		);
		middle_region.feature_target = std::min(4 * m_GridSize.area(), 3000);

		auto& right_region = m_TrackingRegions.emplace_back();
		right_region.region = cv::Rect(
			cv::Point(2 * resolution.width/3, 0),
			cv::Size(resolution.width/3, resolution.height)
		);
		right_region.feature_target = std::min(4 * m_GridSize.area(), 3000);

		restart();
	}

	//-------------------------------------------------------------------------------------

	cv::Point2f FrameTracker::import_next(const cv::UMat& frame)
	{
		LVK_ASSERT(frame.type() == CV_8UC1);

		cv::resize(frame, m_NextFrame, m_TrackingResolution, 0, 0, cv::INTER_NEAREST);

		const cv::Mat sharpening_kernel({3,3}, {
				 0.0f, -0.5f, 0.0f,
				-0.5f, 3.0f, -0.5f,
				 0.0f, -0.5f, 0.0f
		});

		cv::equalizeHist(m_NextFrame, m_NextFrame);
		cv::filter2D(m_NextFrame, m_NextFrame, -1, sharpening_kernel);

		return cv::Point2f(
			static_cast<float>(frame.cols) / m_TrackingResolution.width,
			static_cast<float>(frame.rows) / m_TrackingResolution.height
		);
	}

	//-------------------------------------------------------------------------------------

	Transform FrameTracker::track(const cv::UMat& next_frame)
	{
		LVK_ASSERT(!next_frame.empty() && next_frame.type() == CV_8UC1);

		std::swap(m_PrevFrame, m_NextFrame);
		const auto scaling = import_next(next_frame);

		if(m_FirstFrame)
		{
			m_FirstFrame = false;
			return Transform::Identity();
		}

		// NOTE: With normal translational camera motion, there is a good chance
		// that the next frame's scenery is a subset of the previous frame, but
		// the opposite does not always hold. Especially around the borders of
		// the image. So we perform tracking backwards, then reverse the points
		// when performing transform estimation to keep time flowing forwards.

		m_TrackedPoints.clear();
		for(auto& [region, feature_threshold, feature_target] : m_TrackingRegions)
		{
			m_Features.clear();
			cv::FAST(
				m_NextFrame(region),
				m_Features,
				feature_threshold,
				true
			);

			process_features(m_Features, m_TrackedPoints, region.tl());

			// Dynamically adjust threshold
			if(m_Features.size() > feature_target)
				feature_threshold = lerp(feature_threshold, MAX_FEATURE_THRESHOLD, 0.1);
			else
				feature_threshold = lerp(feature_threshold, MIN_FEATURE_THRESHOLD, 0.1);
		}

		// Return identity transform if we don't have enough trackers
		if(m_TrackedPoints.size() < m_MinMatchThreshold)
			return Transform::Identity();

		// Match tracking points in next frame
		m_MatchedPoints.clear();
		m_MatchStatus.clear();
		cv::calcOpticalFlowPyrLK(
			m_NextFrame,
			m_PrevFrame,
			m_TrackedPoints,
			m_MatchedPoints,
			m_MatchStatus,
			cv::noArray()
		);

		// Filter out non-matches
		fast_filter(m_TrackedPoints, m_MatchedPoints, m_MatchStatus);

		if(m_MatchedPoints.size() < m_MinMatchThreshold)
			return Transform::Identity();

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

		const auto affine_estimate = cv::estimateAffinePartial2D(m_MatchedPoints, m_TrackedPoints);
		if(affine_estimate.empty())
			return Transform::Identity();

		return Transform::FromAffine2D(affine_estimate);
	}

	//-------------------------------------------------------------------------------------

	void FrameTracker::process_features(const std::vector<cv::KeyPoint>& features, std::vector<cv::Point2f>& points, const cv::Point2f& offset)
	{
		// Sort all the best features into the grid
		for(auto& feature : features)
		{
			const auto point = feature.pt + offset;
			const uint32_t grid_x = point.x / m_BlockSize.width;
			const uint32_t grid_y = point.y / m_BlockSize.height;
			const uint32_t index = grid_y * m_GridSize.width + grid_x;

			std::optional<cv::KeyPoint>& curr_feature = m_Grid[index];
			if(!curr_feature.has_value() || curr_feature->response < feature.response)
			{
				curr_feature = feature;
				curr_feature->pt = point;
			}
		}

		// Push all resulting features to the points vector
		for(auto& feature : m_Grid)
		{
			if(feature.has_value())
			{
				points.push_back(feature->pt);
				feature.reset();
			}
		}
	}

	//-------------------------------------------------------------------------------------

	void FrameTracker::restart()
	{
		m_FirstFrame = true;
		for(auto& [region, feature_threshold, feature_target] : m_TrackingRegions)
			feature_threshold = DEFAULT_FEATURE_THRESHOLD;
	}

	//-------------------------------------------------------------------------------------

}
