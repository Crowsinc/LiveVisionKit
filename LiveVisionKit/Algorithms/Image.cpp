//     *************************** LiveVisionKit ****************************
//     Copyright (C) 2022  Sebastian Di Marco (crowsinc.dev@gmail.com)
//
//     This program is free software: you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation, either version 3 of the License, or
//     (at your option) any later version.
//
//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.
//
//     You should have received a copy of the GNU General Public License
//     along with this program.  If not, see <https://www.gnu.org/licenses/>.
//     **********************************************************************

#include "Image.hpp"

#include "OpenCL/Kernels.hpp"
#include "Directives.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

    void scale(const cv::UMat& src, cv::UMat& dst, const cv::Size& size)
    {
        LVK_ASSERT(size.width > 0 && size.height > 0);
        LVK_ASSERT(src.type() == CV_8UC3);
        LVK_ASSERT(!src.empty());

        // Make sure the output is initialized to the output size.
        dst.create(size, CV_8UC3);

        static cv::ocl::Kernel kernel = ocl::load_kernel("easu", ocl::src::fsr_source);
        LVK_ASSERT(!kernel.empty());

        // Get raw size of the source buffer, without ROIs
        cv::Size raw_size; cv::Point _;
        src.locateROI(raw_size, _);

        size_t local_size[2] = {8,8};
        size_t global_size[2] = {static_cast<size_t>(dst.cols), static_cast<size_t>(dst.rows)};

        kernel.args(
            cv::ocl::KernelArg::ReadOnlyNoSize(src),
            cv::ocl::KernelArg::WriteOnlyNoSize(dst),
            cv::Vec2i{raw_size.width, raw_size.height},
            cv::Vec2f{
                static_cast<float>(src.cols) / static_cast<float>(dst.cols),
                static_cast<float>(src.rows) / static_cast<float>(dst.rows)
            },
            cv::Vec2f{0,0}
        ).run(2, global_size, local_size, true);
    }

//---------------------------------------------------------------------------------------------------------------------

    void remap(const cv::UMat& src, cv::UMat& dst, const cv::UMat& offsets)
    {
        LVK_ASSERT(offsets.size() == src.size());
        LVK_ASSERT(offsets.type() == CV_32FC2);
        LVK_ASSERT(src.type() == CV_8UC3);
        LVK_ASSERT(!offsets.empty());
        LVK_ASSERT(!src.empty());

        // Make sure the output is initialized.
        dst.create(src.size(), CV_8UC3);

        static cv::ocl::Kernel kernel = ocl::load_kernel("easu_warp", ocl::src::fsr_source);
        LVK_ASSERT(!kernel.empty());

        // Get raw size of the source buffer, without ROIs
        cv::Size raw_size; cv::Point _;
        src.locateROI(raw_size, _);

        size_t local_size[2] = {8,8};
        size_t global_size[2] = {static_cast<size_t>(dst.cols), static_cast<size_t>(dst.rows)};

        kernel.args(
            cv::ocl::KernelArg::ReadOnlyNoSize(src),
            cv::ocl::KernelArg::WriteOnlyNoSize(dst),
            cv::ocl::KernelArg::ReadOnlyNoSize(offsets),
            cv::Vec2i{raw_size.width, raw_size.height},
            cv::Vec2f{1, 1}
        ).run(2, global_size, local_size, true);
    }

//---------------------------------------------------------------------------------------------------------------------

}