#include "FrameTracker.hpp"

#include <algorithm>

#include "../Utility/Algorithm.hpp"

#include "../Diagnostics/Assert.hpp"

namespace lvk
{

	//-------------------------------------------------------------------------------------

	FrameTracker::FrameTracker(const Properties properties)
		: m_Properties(properties),
		  m_MatchThreshold(properties.max_trackers * properties.match_proportion),
		  m_TrackThreshold(std::max(properties.tracker_quality * 255, 1.0)),
		  m_PrevFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_NextFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY)
	{
		LVK_ASSERT(properties.max_trackers > 0);
		LVK_ASSERT(properties.resolution.width > 0 && properties.resolution.height > 0);
		LVK_ASSERT(properties.tracker_quality >= 0 && properties.tracker_quality <= 1.0);
		LVK_ASSERT(properties.match_proportion * properties.max_trackers >= 3 && properties.match_proportion <= 1.0);

		reset();

		m_KeyPoints.reserve(2 * m_Properties.max_trackers);
		m_TrackPoints.reserve(m_Properties.max_trackers);
		m_MatchedPoints.reserve(m_Properties.max_trackers);
	}

	//-------------------------------------------------------------------------------------

	Transform FrameTracker::track(const cv::UMat& next_frame)
	{
		LVK_ASSERT(!next_frame.empty() && next_frame.type() == CV_8UC1);

		import_next(next_frame);

		// Can't track until we have imported at least 2 frames
		if(frames_tracked() < 2)
		{
			std::swap(m_PrevFrame, m_NextFrame);
			return Transform::Identity();
		}

		m_KeyPoints.clear();
		m_TrackPoints.clear();
		m_MatchedPoints.clear();
		m_MatchStatus.clear();

		// Find tracking points for previous frame
		cv::FAST(
			m_PrevFrame,
			m_KeyPoints,
			m_TrackThreshold,
			true
		);

		cv::KeyPointsFilter::retainBest(m_KeyPoints, m_Properties.max_trackers);
		cv::KeyPoint::convert(m_KeyPoints, m_TrackPoints, {});

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
		const double x_scale = static_cast<float>(next_frame.cols) / m_Properties.resolution.width;
		const double y_scale = static_cast<float>(next_frame.rows) / m_Properties.resolution.height;
		for(size_t i = 0; i < m_TrackPoints.size(); i++)
		{
			m_TrackPoints[i].x *= x_scale;
			m_TrackPoints[i].y *= y_scale;
			m_MatchedPoints[i].y *= y_scale;
			m_MatchedPoints[i].x *= x_scale;
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

	void FrameTracker::import_next(const cv::UMat& frame)
	{
		LVK_ASSERT(frame.type() == CV_8UC1);

		cv::resize(frame, m_NextFrame, m_Properties.resolution, 0, 0, cv::INTER_NEAREST);
		m_FrameCount++;

		const cv::Mat sharpening_kernel({3,3}, {
				 0.0f, -0.5f, 0.0f,
				-0.5f, 3.0f, -0.5f,
				 0.0f, -0.5f, 0.0f
		});

		cv::equalizeHist(m_NextFrame, m_NextFrame);
		cv::filter2D(m_NextFrame, m_NextFrame, -1, sharpening_kernel);
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


}
