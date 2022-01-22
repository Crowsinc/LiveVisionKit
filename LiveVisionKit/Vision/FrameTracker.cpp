#include "FrameTracker.hpp"

#include <algorithm>
#include <tuple>

#include "../Utility/Algorithm.hpp"

namespace lvk
{

	//-------------------------------------------------------------------------------------

	FrameTracker::FrameTracker(const Properties properties)
		: m_Properties(properties),
		  m_MatchThreshold(properties.max_trackers * properties.tracker_threshold),
		  m_PrevFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_NextFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_CumulativeTransform(Transform::Identity())
	{
		reset();

		m_TrackPoints.reserve(m_Properties.max_trackers);
		m_MatchedPoints.reserve(m_Properties.max_trackers);
	}

	//-------------------------------------------------------------------------------------

	Transform FrameTracker::track(const cv::UMat& next_frame)
	{
		//TODO: Assert 1 component & consistent frame size

		import_next(next_frame);

		// Can't track until we have imported at least 2 frames
		if(frames() < 2)
		{
			// Determine the motion scale based on the size of the first frame
			m_MotionScale.x = static_cast<float>(next_frame.cols) / m_Properties.tracking_size.width;
			m_MotionScale.y = static_cast<float>(next_frame.rows) / m_Properties.tracking_size.width;

			// Round-robin the buffers so we can re-use the internal frame we just imported.
			std::swap(m_PrevFrame, m_NextFrame);

			return Transform::Identity();
		}

		m_MatchStatus.clear();
		m_TrackPoints.clear();
		m_MatchedPoints.clear();

		// Find tracking points for previous frame
		cv::goodFeaturesToTrack(
				m_PrevFrame,
				m_TrackPoints,
				m_Properties.max_trackers,
				m_Properties.tracker_quality,
				m_Properties.min_tracker_distance
		);

		// Match tracking points in next frame
		cv::calcOpticalFlowPyrLK(
				m_PrevFrame,
				m_NextFrame,
				m_TrackPoints,
				m_MatchedPoints,
				m_MatchedPoints,
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
			m_TrackPoints[i].x *= m_MotionScale.x;
			m_TrackPoints[i].y *= m_MotionScale.y;
			m_MatchedPoints[i].x *= m_MotionScale.x;
			m_MatchedPoints[i].y *= m_MotionScale.y;
		}

		// Estimate the affine transform required to match the track points to the matched points
		// TODO: test USAC if the partial affine function gets support.
		const auto affine_estimate = cv::estimateAffinePartial2D(m_TrackPoints, m_MatchedPoints);

		// If nno estimate was found, return identity transform.
		if(affine_estimate.empty())
			return Transform::Identity();

		const auto estimated_transform = Transform::FromAffine2D(affine_estimate);

		m_CumulativeTransform += estimated_transform;

		// Only add the change in scale, relative to 1.0
		m_CumulativeTransform.scale -= 1.0;

		return estimated_transform;
	}

	//-------------------------------------------------------------------------------------

	void FrameTracker::import_next(const cv::UMat& frame)
	{
		// The internal frame is made smaller to speed up computations and lightly sharpened
		// to remove blurriness and increase the number of detected tracking points later.
		//
		// TODO: Test more transformations other than the sharpening pass.

		cv::resize(frame, m_NextFrame, m_Properties.tracking_size, 0, 0, cv::INTER_NEAREST);
		m_FrameCount++;

		const cv::Mat light_sharpening_kernel({3,3}, {
				 0.0f, -0.5f, 0.0f,
				-0.5f, 3.0f, -0.5f,
				 0.0f, -0.5f, 0.0f
		});

		cv::filter2D(m_NextFrame, m_NextFrame, -1, light_sharpening_kernel);
	}

	//-------------------------------------------------------------------------------------

	void FrameTracker::reset()
	{
		m_FrameCount = 0;
		m_MotionScale = {1, 1};
		m_CumulativeTransform = Transform::Identity();
	}

	//-------------------------------------------------------------------------------------

	Transform FrameTracker::cumulative() const
	{
		return m_CumulativeTransform;
	}

	//-------------------------------------------------------------------------------------

	uint64_t FrameTracker::frames() const
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
