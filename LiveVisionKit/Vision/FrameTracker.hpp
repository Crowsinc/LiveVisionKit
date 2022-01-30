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
			// How much a feature must stand out from its surroundings
			// to be considered as a tracking point; where 0.0 is the
			// lowest threshold, and 1.0 is the highest.
			double tracker_quality = 0.2;

			// The maximum amount of tracking points to use per frame.
			// A higher amount usually leads more robust tracking at the
			// cost of computational performance.
			uint32_t max_trackers = 3000;

			// The minimum matched tracking points required for transform
			// estimation as a percentage of the max trackers. If the
			// threshold is not met, the tracker defaults to zero motion.
			double match_proportion = 0.2;

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

		const Properties& properties() const;

	private:

		const Properties m_Properties;
		const uint32_t m_MatchThreshold;
		const uint32_t m_TrackThreshold;

		std::vector<cv::KeyPoint> m_KeyPoints;
		std::vector<cv::Point2f> m_TrackPoints;
		std::vector<cv::Point2f> m_MatchedPoints;
		std::vector<uint8_t> m_MatchStatus;
		cv::UMat m_PrevFrame, m_NextFrame;
		uint64_t m_FrameCount;

		void import_next(const cv::UMat& frame);

	};

}
