#pragma once

#include <obs/obs-module.h>
#include <opencv2/opencv.hpp>

namespace lvk
{

	// Converts OBS frame to YUV UMat
	bool extract_frame(const obs_source_frame* src, cv::UMat& dst);

	// Converts YUV UMat back to OBS frame
	void insert_frame(const cv::UMat& src, obs_source_frame* dst);

}

bool operator<<(cv::UMat& dst, const obs_source_frame* src);

void operator>>(const cv::UMat& src, obs_source_frame* dst);
