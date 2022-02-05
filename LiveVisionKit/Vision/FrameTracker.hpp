#pragma once


#include <opencv2/opencv.hpp>
#include <optional>

#include "../Math/Transform.hpp"

namespace lvk
{

	class FrameTracker
	{
	public:

		FrameTracker(const float estimation_threshold = 0.05, const cv::Size resolution = cv::Size(960,540), const cv::Size block_size = cv::Size(30, 30));

		Transform track(const cv::UMat& next_frame);

		void restart();

	private:

		struct TrackingRegion
		{
			cv::Rect region;
			double feature_threshold;
			uint32_t feature_target;
		};

		const cv::Size m_TrackingResolution;
		const cv::Size m_BlockSize, m_GridSize;
		const uint32_t m_MinMatchThreshold;

		std::vector<cv::KeyPoint> m_Features;
		std::vector<TrackingRegion> m_TrackingRegions;
		std::vector<std::optional<cv::KeyPoint>> m_Grid;

		std::vector<cv::Point2f> m_TrackedPoints;
		std::vector<cv::Point2f> m_MatchedPoints;
		std::vector<uint8_t> m_MatchStatus;

		cv::UMat m_PrevFrame, m_NextFrame;
		bool m_FirstFrame;

		void process_features(const std::vector<cv::KeyPoint>& features, std::vector<cv::Point2f>& points, const cv::Point2f& offset);

		cv::Point2f import_next(const cv::UMat& frame);

	};

}
