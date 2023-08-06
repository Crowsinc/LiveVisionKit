//    *************************** LiveVisionKit ****************************
//    Copyright (C) 2022  Sebastian Di Marco (crowsinc.dev@gmail.com)
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.
// 	  **********************************************************************

#pragma once

#include <opencv2/opencv.hpp>

#include "Data/VideoFrame.hpp"

namespace lvk
{

	struct CameraParameters
	{
		cv::Mat camera_matrix = cv::Mat::eye(3,3, CV_64FC1);
		std::vector<double> distortion_coefficients;
	};

	class CameraCalibrator
	{
	public:

        explicit CameraCalibrator(const cv::Size& pattern_size) ;

		bool feed(const VideoFrame& frame);

        bool feed_and_draw(VideoFrame& frame);

		CameraParameters calibrate(const uint32_t square_size = 1) const;

        uint32_t calibration_frames() const;

        void reset();

	private:

		const cv::Size m_PatternSize;

		cv::Size m_ImageSize;
		VideoFrame m_DetectionFrame;
		std::vector<std::vector<cv::Point2f>> m_ImagePoints;

	};

}
