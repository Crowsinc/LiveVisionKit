#include "FrameTracker.hpp"

#include <algorithm>

#include "../Math/Math.hpp"
#include "../Utility/Algorithm.hpp"
#include "../Diagnostics/Assert.hpp"

namespace lvk
{

	//-------------------------------------------------------------------------------------

	FrameTracker::FrameTracker(const Properties properties)
		: m_Properties(properties),
		  m_MatchThreshold(properties.max_trackers * properties.min_matches),
		  m_TrackThreshold(std::max(properties.tracker_quality * 255, 1.0)),
		  m_MaxRegionTrackers(properties.max_trackers / properties.grid.area()),
		  m_PrevFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_NextFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY)
	{
		LVK_ASSERT(properties.max_trackers > 0);
		LVK_ASSERT(properties.resolution.width > 0 && properties.resolution.height > 0);
		LVK_ASSERT(properties.grid.width > 0 && properties.grid.height > 0);
		LVK_ASSERT(properties.grid.width < properties.resolution.width/2 && properties.grid.height < properties.resolution.height/2);
		LVK_ASSERT(properties.tracker_quality >= 0 && properties.tracker_quality <= 1.0);
		LVK_ASSERT(properties.crop_proportion >= 0 && properties.crop_proportion <= 0.25);
		LVK_ASSERT(properties.min_matches * properties.max_trackers >= 3 && properties.min_matches <= 1.0);

		reset();

		m_KeyPoints.reserve(2 * m_Properties.max_trackers);
		m_TrackPoints.reserve(m_Properties.max_trackers);
		m_MatchedPoints.reserve(m_Properties.max_trackers);

		// Pre-compute grid regions

		const cv::Rect crop_region = crop(
				properties.resolution,
				properties.crop_proportion
		);

		const cv::Size cell_size(
			crop_region.width / properties.grid.width,
			crop_region.height / properties.grid.height
		);

		for(int y = 0; y < properties.grid.height; y++)
			for(int x = 0; x < properties.grid.width; x++)
				m_GridRegions.emplace_back(
					cv::Point(
						x * cell_size.width + crop_region.x,
						y * cell_size.height + crop_region.y
					),
					cell_size
				);
	}

	//-------------------------------------------------------------------------------------

	Transform FrameTracker::track(const cv::UMat& next_frame)
	{
		LVK_ASSERT(!next_frame.empty() && next_frame.type() == CV_8UC1);

		const auto scaling = import_next(next_frame);

		// Can't track until we have imported at least 2 frames
		if(frames_tracked() < 2)
		{
			std::swap(m_PrevFrame, m_NextFrame);
			return Transform::Identity();
		}

		m_TrackPoints.clear();
		m_MatchedPoints.clear();
		m_MatchStatus.clear();

		// Find tracking points for each grid region of previous frame
		for(const auto& region : m_GridRegions)
		{
			m_KeyPoints.clear();

			cv::FAST(
				m_PrevFrame(region),
				m_KeyPoints,
				m_TrackThreshold,
				true
			);

			cv::KeyPointsFilter::removeDuplicated(m_KeyPoints);
			cv::KeyPointsFilter::retainBest(m_KeyPoints, m_MaxRegionTrackers);
			for(const auto& keypoint : m_KeyPoints)
				m_TrackPoints.push_back(keypoint.pt + cv::Point2f(region.x, region.y));
		}

		// Return identity transform if we don't have enough trackers
		if(m_TrackPoints.size() < m_MatchThreshold)
		{
			std::swap(m_PrevFrame, m_NextFrame);
			return Transform::Identity();
		}

		// Match tracking points in next frame
		cv::calcOpticalFlowPyrLK(
			m_PrevFrame,
			m_NextFrame,
			m_TrackPoints,
			m_MatchedPoints,
			m_MatchStatus,
			cv::noArray()
		);

		// Round-robin the buffers so we can re-use the internal frame we just imported.
		std::swap(m_PrevFrame, m_NextFrame);

		// Filter out non-matches
		fast_filter(m_TrackPoints, m_MatchedPoints, m_MatchStatus);

		// Return identity transform if we don't have enough matches
		if(m_MatchedPoints.size() < m_MatchThreshold)
			return Transform::Identity();

		// Re-scale all the points to original frame size otherwise the motion won't correspond
		for(size_t i = 0; i < m_TrackPoints.size(); i++)
		{
			auto& tracked_point = m_TrackPoints[i];
			tracked_point.x *= scaling.x;
			tracked_point.y *= scaling.y;

			auto& matched_point = m_MatchedPoints[i];
			matched_point.x *= scaling.x;
			matched_point.y *= scaling.y;
		}

		// Estimate the affine transform required to match the track points to the matched points
		// TODO: test USAC if the partial affine function gets support.
		const auto affine_estimate = cv::estimateAffinePartial2D(m_TrackPoints, m_MatchedPoints);

		// If nno estimate was found, return identity transform.
		if(affine_estimate.empty())
			return Transform::Identity();

		return Transform::FromAffine2D(affine_estimate);
	}

	//-------------------------------------------------------------------------------------

	cv::Point2f FrameTracker::import_next(const cv::UMat& frame)
	{
		LVK_ASSERT(frame.type() == CV_8UC1);

		cv::resize(frame, m_NextFrame, m_Properties.resolution, 0, 0, cv::INTER_NEAREST);

		const cv::Mat sharpening_kernel({3,3}, {
				 0.0f, -0.5f, 0.0f,
				-0.5f, 3.0f, -0.5f,
				 0.0f, -0.5f, 0.0f
		});

		cv::equalizeHist(m_NextFrame, m_NextFrame);
		cv::filter2D(m_NextFrame, m_NextFrame, -1, sharpening_kernel);

		m_FrameCount++;

		return cv::Point2f(
			static_cast<float>(frame.cols) / m_Properties.resolution.width,
			static_cast<float>(frame.rows) / m_Properties.resolution.height
		);
	}

	//-------------------------------------------------------------------------------------

	void FrameTracker::reset()
	{
		m_FrameCount = 0;
	}

	//-------------------------------------------------------------------------------------

	uint64_t FrameTracker::frames_tracked() const
	{
		return m_FrameCount;
	}

	//-------------------------------------------------------------------------------------

	const std::vector<cv::Point2f>& FrameTracker::tracking_points() const
	{
		return m_TrackPoints;
	}

	//-------------------------------------------------------------------------------------

	const std::vector<cv::Point2f>& FrameTracker::matched_points() const
	{
		return m_MatchedPoints;
	}

	//-------------------------------------------------------------------------------------

	const FrameTracker::Properties& FrameTracker::properties() const
	{
		return m_Properties;
	}

	//-------------------------------------------------------------------------------------

}
