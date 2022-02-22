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

namespace lvk::draw
{
	const cv::Scalar YUV_MAGENTA(105, 212, 234);
	const cv::Scalar YUV_GREEN(149, 43, 21);
	const cv::Scalar YUV_BLUE(29, 255, 107);
	const cv::Scalar YUV_RED(76, 84, 255);

	const cv::Scalar BGR_MAGENTA(255, 0, 255);
	const cv::Scalar BGR_GREEN(0, 255, 0);
	const cv::Scalar BGR_BLUE(255, 0, 0);
	const cv::Scalar BGR_RED(0, 0, 255);

	template<typename T>
	void plot_markers(
		cv::UMat& dst,
		const std::vector<cv::Point_<T>>& markers,
		const cv::Scalar& color,
		const cv::MarkerTypes type = cv::MarkerTypes::MARKER_CROSS,
		const int size = 10,
		const int thickness = 2
	);

	template<typename T>
	void text(
		cv::UMat& dst,
		const std::string& text,
		const cv::Point_<T>& position,
		const cv::Scalar& color,
		const double font_scale = 1.5,
		const int font_thickness = 2,
		const cv::HersheyFonts font = cv::FONT_HERSHEY_DUPLEX
	);

	template<typename T>
	void rect(
		cv::UMat& dst,
		const cv::Rect_<T>& rect,
		const cv::Scalar& color,
		const int thickness = 2
	);

}

#include "Drawing.tpp"
