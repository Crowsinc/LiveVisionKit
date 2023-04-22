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

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline void draw_rect(
        cv::UMat& dst,
        const cv::Rect_<T>& rect,
        const cv::Scalar& color,
        const int thickness
    )
    {
        cv::rectangle(dst, rect, color, thickness);
    }

//---------------------------------------------------------------------------------------------------------------------

    inline void draw_grid(
        cv::UMat& dst,
        const cv::Size& grid,
        const cv::Scalar& color,
        const int thickness
    )
    {
        thread_local cv::UMat colour_mask(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

        // NOTE: Individually drawing lots of lines on a UMat is very inefficient.
        // Instead, draw the grid lines to a mask and apply them in bulk to the UMat.

        thread_local cv::Mat draw_mask;
        draw_mask.create(dst.size(), CV_8UC1);

        const auto cell_width = static_cast<float>(dst.cols) / static_cast<float>(grid.width);
        const auto cell_height = static_cast<float>(dst.rows) / static_cast<float>(grid.height);

        draw_mask.forEach<uint8_t>([&](uint8_t& mask, const int coord[]){
            mask = std::fmod(coord[1], cell_width) < thickness || std::fmod(coord[0], cell_height) < thickness;
        });

        draw_mask.copyTo(colour_mask);
        dst.setTo(color, colour_mask);
    }

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	inline void draw_markers(
		cv::UMat& dst,
		const cv::Scalar& color,
		const std::vector<cv::Point_<T>>& markers,
        const cv::Size2f& position_scaling,
		const cv::MarkerTypes marker_type,
		const int marker_size,
		const int marker_thickness
	)
	{
		thread_local cv::UMat colour_mask(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

		// NOTE: Individually drawing lots of points on a UMat is very inefficient.
		// Instead, draw the points to a mask and apply them in bulk to the UMat.

		thread_local cv::Mat draw_mask;
		draw_mask.create(dst.size(), CV_8UC1);
		draw_mask.setTo(cv::Scalar(0));

        // TODO: paralellize?
		for(const auto& point : markers)
        {
            cv::Point_<T> scaled_point(
                point.x * position_scaling.width,
                point.y * position_scaling.height
            );

			cv::drawMarker(draw_mask, scaled_point, color, marker_type, marker_size, marker_thickness);
        }

		draw_mask.copyTo(colour_mask);
		dst.setTo(color, colour_mask);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	inline void draw_text(
		cv::UMat& dst,
		const std::string& text,
		const cv::Point_<T>& position,
		const cv::Scalar& color,
		const double font_scale,
		const int font_thickness,
		const cv::HersheyFonts font
	)
	{
		cv::putText(
			dst,
			text,
			position,
			font,
			font_scale,
			color,
			font_thickness
		);
	}

//---------------------------------------------------------------------------------------------------------------------

}
