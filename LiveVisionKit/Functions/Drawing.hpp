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

// COLOUR CONSTANTS
namespace lvk::rgb
{
	const cv::Scalar BLACK(0, 0, 0);
	const cv::Scalar WHITE(255, 255, 255);
	const cv::Scalar MAGENTA(255, 0, 255);
	const cv::Scalar GREEN(0, 255, 0);
	const cv::Scalar BLUE(0, 0, 255);
	const cv::Scalar RED(255, 0, 0);
}
namespace lvk::bgr
{
    const cv::Scalar BLACK(0, 0, 0);
    const cv::Scalar WHITE(255, 255, 255);
    const cv::Scalar MAGENTA(255, 0, 255);
    const cv::Scalar GREEN(0, 255, 0);
    const cv::Scalar BLUE(255, 0, 0);
    const cv::Scalar RED(0, 0, 255);
}
namespace lvk::yuv
{
    const cv::Scalar BLACK(0, 128, 128);
    const cv::Scalar WHITE(255, 0, 0);
    const cv::Scalar MAGENTA(105, 212, 234);
    const cv::Scalar GREEN(149, 43, 21);
    const cv::Scalar BLUE(29, 255, 107);
    const cv::Scalar RED(76, 84, 255);
}
namespace lvk::gray
{
	// Taken from the Y plane of the YUV colours.
	const cv::Scalar BLACK(0);
	const cv::Scalar WHITE(255);
	const cv::Scalar MAGENTA(105);
	const cv::Scalar GREEN(149);
	const cv::Scalar BLUE(29);
	const cv::Scalar RED(76);
}

// COLOURS INDEXED BY FORMAT
// TODO: find a better way to implement this
namespace lvk::col
{
	// Formats: BGR, BGRA, RGB, RGBA, YUV, GRAY
	const cv::Scalar BLACK[] = {bgr::BLACK, bgr::BLACK, rgb::BLACK, rgb::BLACK, yuv::BLACK, gray::BLACK};
	const cv::Scalar WHITE[] = {bgr::WHITE, bgr::WHITE, rgb::WHITE, rgb::WHITE, yuv::WHITE, gray::WHITE};
	const cv::Scalar MAGENTA[] = {bgr::MAGENTA, bgr::MAGENTA, rgb::MAGENTA, rgb::MAGENTA, yuv::MAGENTA, gray::MAGENTA};
	const cv::Scalar GREEN[] = {bgr::GREEN, bgr::GREEN, rgb::GREEN, rgb::GREEN, yuv::GREEN, gray::GREEN};
	const cv::Scalar BLUE[] = {bgr::BLUE, bgr::BLUE, rgb::BLUE, rgb::BLUE, yuv::BLUE, gray::BLUE};
	const cv::Scalar RED[] = {bgr::RED, bgr::RED, rgb::RED, rgb::RED, yuv::RED, gray::RED};
}

// DRAWING FUNCTIONS
namespace lvk
{

    template<typename T>
    void draw_rect(
        cv::UMat& dst,
        const cv::Rect_<T>& rect,
        const cv::Scalar& color,
        const int thickness = 2
    );

    void draw_grid(
        cv::UMat& dst,
        const cv::Size& grid,
        const cv::Scalar& color,
        const int thickness = 2
    );

    template<typename T>
    void draw_points(
        cv::UMat& dst,
        const std::vector<cv::Point_<T>>& points,
        const cv::Scalar& color,
        const int32_t point_size = 10,
        const cv::Size2f& coord_scaling = {1, 1}
    );

    template<typename T>
    void draw_crosses(
        cv::UMat& dst,
        const std::vector<cv::Point_<T>>& points,
        const cv::Scalar& color,
        const int32_t cross_size = 10,
        const int32_t cross_thickness = 3,
        const cv::Size2f& coord_scaling = {1, 1}
    );

	template<typename T>
	void draw_text(
		cv::UMat& dst,
		const std::string& text,
		const cv::Point_<T>& position,
		const cv::Scalar& color,
		const double font_scale = 1.5,
		const int font_thickness = 2,
		const cv::HersheyFonts font = cv::FONT_HERSHEY_DUPLEX
	);

}

#include "Drawing.tpp"
