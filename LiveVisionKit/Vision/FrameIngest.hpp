#pragma once

#include <obs/obs-source.h>
#include <opencv2/core.hpp>

namespace lvk
{

	void ingest(const obs_source_frame& frame, cv::Mat& dst, bool clone = false);

	void operator<<(cv::Mat& dst, const obs_source_frame& frame);

}
