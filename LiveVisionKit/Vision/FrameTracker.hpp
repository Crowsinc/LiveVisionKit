#pragma once

#include "../Math/Transform.hpp"

#include <opencv2/opencv.hpp>

namespace lvk
{

	class FrameTracker
	{
	public:

		struct Properties
		{
			// The maximum amount of tracking points to use per frame.
			// A higher amount usually leads more robust tracking at the
			// cost of computational performance.
			uint32_t max_trackers = 500;

			// The minimum allowable distance between each tracking point.
			// A lower amount leads to an increase in discovered tracking
			// points per frame. But potentially leads to spatial bias in
			// the tracking.
			uint32_t min_tracker_distance = 20;

			// The quality threshold for tracking points as a percentage
			// of the highest quality tracker found each track operation.
			// A higher amount leads to fewer, high quality points.
			double tracker_quality = 0.1;

			// The minimum matched tracking points required for transform
			// estimation as a percentage of the max trackers. If the
			// threshold is not met, the tracker defaults to zero motion.
			double match_proportion = 0.1;

			// The internal resolution used for the tracking. A lower
			// resolution leads to a decrease in tracking points, but
			// much faster computation speed.
			cv::Size resolution = cv::Size(960, 540);
		};

	public:

		FrameTracker(const Properties properties);

		Transform track(const cv::UMat& next_frame);

		uint64_t frames_tracked() const;

		void reset();

		const std::vector<cv::Point2f>& tracking_points() const;

		const std::vector<cv::Point2f>& matched_points() const;

	private:

		const Properties m_Properties;
		const uint32_t m_MatchThreshold;

		std::vector<cv::Point2f> m_TrackPoints, m_MatchedPoints;
		std::vector<uint8_t> m_MatchStatus;
		cv::UMat m_PrevFrame, m_NextFrame;
		uint64_t m_FrameCount;

		void import_next(const cv::UMat& frame);

	};

}
