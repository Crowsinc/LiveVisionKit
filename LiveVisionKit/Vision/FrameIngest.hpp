#pragma once

#include <obs/obs-module.h>
#include <opencv2/core.hpp>

namespace lvk
{

	bool ingest(const obs_source_frame& frame, cv::UMat& dst);

}

bool operator<<(cv::UMat& dst, const obs_source_frame& frame);
