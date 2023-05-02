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

#include "Directives.hpp"
#include "OpenCL/Kernels.hpp"

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
        LVK_ASSERT(dst.type() == CV_8UC3);
        LVK_ASSERT(thickness >= 1);
        LVK_ASSERT(!dst.empty());

        // Create FSR RCAS kernel
        static auto program = ocl::load_program("draw", ocl::src::drawing_source);
        thread_local cv::ocl::Kernel kernel("grid", program);
        LVK_ASSERT(!program.empty() && !kernel.empty());

        // Find cell size of the grid
        const float cell_width = static_cast<float>(dst.cols) / static_cast<float>(grid.width);
        const float cell_height = static_cast<float>(dst.rows) / static_cast<float>(grid.height);

        // Find optimal work sizes for the 2D dst buffer.
        size_t global_work_size[3], local_work_size[3];
        ocl::optimal_groups(dst, global_work_size, local_work_size);

        // Run the kernel in async mode.
        kernel.args(
            cv::ocl::KernelArg::WriteOnly(dst),
            cell_width, cell_height, thickness,
            cv::Vec4b{
                static_cast<uint8_t>(color[0]),
                static_cast<uint8_t>(color[1]),
                static_cast<uint8_t>(color[2]),
                0 // NOTE: 4th component is unused
            }
        ).run_(2, global_work_size, local_work_size, false);

        // Create next kernel while the last one runs.
        kernel.create("grid", program);
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline void draw_points(
        cv::UMat& dst,
        const std::vector<cv::Point_<T>>& points,
        const cv::Scalar& color,
        const int32_t point_size,
        const cv::Scalar& point_scaling
    )
    {
        LVK_ASSERT(point_scaling[0] >= 0 && point_scaling[1] >= 0);
        LVK_ASSERT(dst.type() == CV_8UC3);
        LVK_ASSERT(point_size >= 1);
        LVK_ASSERT(!dst.empty());

        if(points.empty())
            return;

        // Create FSR RCAS kernel
        static auto program = ocl::load_program("draw", ocl::src::drawing_source);
        thread_local cv::ocl::Kernel kernel("points", program);
        LVK_ASSERT(!program.empty() && !kernel.empty());

        // Upload and scale points to 32bit int image coords.
        thread_local cv::UMat staging_buffer, points_buffer;
        cv::Mat(points, false).copyTo(staging_buffer);
        cv::multiply(staging_buffer, point_scaling, points_buffer, 1, CV_32S);

        // Find optimal work sizes for the 1D points buffer.
        size_t global_work_size[3], local_work_size[3];
        ocl::optimal_groups(points_buffer, global_work_size, local_work_size);

        // Run the kernel in async mode.
        kernel.args(
            cv::ocl::KernelArg::ReadOnly(points_buffer),
            cv::ocl::KernelArg::WriteOnly(dst),
            (point_size + 1) / 2,
            cv::Vec4b{
                static_cast<uint8_t>(color[0]),
                static_cast<uint8_t>(color[1]),
                static_cast<uint8_t>(color[2]),
                0 // NOTE: 4th component is unused
            }
        ).run_(1, global_work_size, local_work_size, false);

        // Create next kernel while the last one runs.
        kernel.create("points", program);
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
