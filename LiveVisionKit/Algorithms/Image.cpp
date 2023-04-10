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

    void upscale(const cv::UMat& src, cv::UMat& dst, const cv::Size& size, const bool yuv)
    {
        LVK_ASSERT(size.width >= src.cols && size.height >= src.rows);
        LVK_ASSERT(src.cols > 8 && src.rows > 8);
        LVK_ASSERT(src.type() == CV_8UC3);
        LVK_ASSERT(!src.empty());

        // FSR program has yuv and bgr versions for different luma calculations.
        static auto program_yuv = ocl::load_program("fsr", ocl::src::fsr_source, "-D YUV_INPUT");
        static auto program_bgr = ocl::load_program("fsr", ocl::src::fsr_source);
        LVK_ASSERT(!program_yuv.empty() && !program_bgr.empty());

        // Create FSR EASU kernel
        thread_local cv::ocl::Kernel kernel;
        thread_local bool kernel_is_yuv = yuv;
        if(kernel.empty() || kernel_is_yuv != yuv)
        {
            kernel.create("easu_scale", yuv ? program_yuv : program_bgr);
        }

        // Allocate the output.
        dst.create(size, CV_8UC3);

        // The FSR kernel does not perform bounds to minimize the performance
        // impact of not being able to usen image sampler. Instead, we shrink
        // the source area so that the 12 tap kernel cannot go out of bounds.
        auto safe_src = src(cv::Rect{4, 4, src.cols - 8, src.rows - 8});

        // Set the local and global work sizes for the kernel. Note that we must
        // enforce that the global work size is a multiple of the local work size.
        size_t local_work_size[2] = {8,8}, global_work_size[2] = {
            static_cast<size_t>(std::ceil(static_cast<float>(dst.cols) / 8.0f)) * 8,
            static_cast<size_t>(std::ceil(static_cast<float>(dst.rows) / 8.0f)) * 8
        };

        // Run the kernel in async mode.
        kernel.args(
            cv::ocl::KernelArg::ReadOnlyNoSize(safe_src),
            cv::Vec2f{
                static_cast<float>(safe_src.cols) / static_cast<float>(dst.cols),
                static_cast<float>(safe_src.rows) / static_cast<float>(dst.rows)
            },
            cv::ocl::KernelArg::WriteOnlyNoSize(dst),
            cv::Vec2i{dst.cols, dst.rows}
        ).run_(2, global_work_size, local_work_size, false);

        // Create next kernel while the last one runs.
        kernel.create("easu_scale", yuv ? program_yuv : program_bgr);
        kernel_is_yuv = yuv;
    }

//---------------------------------------------------------------------------------------------------------------------

    void remap(const cv::UMat& src, cv::UMat& dst, const cv::UMat& offsets, const bool yuv)
    {
        LVK_ASSERT(src.cols > 8 && src.rows > 8);
        LVK_ASSERT(offsets.size() == src.size());
        LVK_ASSERT(offsets.type() == CV_32FC2);
        LVK_ASSERT(src.type() == CV_8UC3);
        LVK_ASSERT(!offsets.empty());
        LVK_ASSERT(!src.empty());

        // FSR program has yuv and bgr versions for different luma calculations.
        static auto program_yuv = ocl::load_program("fsr", ocl::src::fsr_source, "-D YUV_INPUT");
        static auto program_bgr = ocl::load_program("fsr", ocl::src::fsr_source);
        LVK_ASSERT(!program_yuv.empty() && !program_bgr.empty());

        // Create FSR EASU kernel
        thread_local cv::ocl::Kernel kernel;
        thread_local bool kernel_is_yuv = yuv;
        if(kernel.empty() || kernel_is_yuv != yuv)
        {
            kernel.create("easu_remap", yuv ? program_yuv : program_bgr);
        }

        // Allocate the output.
        dst.create(src.size(), CV_8UC3);

        // The FSR kernel does not perform bounds to minimize the performance
        // impact of not being able to usen image sampler. Instead, we shrink
        // the source area so that the 12 tap kernel cannot go out of bounds.
        const cv::Rect safe_region(1, 1, src.cols - 5, src.rows - 5);

        // Set the local and global work sizes for the kernel. Note that we must
        // enforce that the global work size is a multiple of the local work size.
        size_t local_work_size[2] = {8,8}, global_work_size[2] = {
            static_cast<size_t>(std::ceil(static_cast<float>(dst.cols) / 8.0f)) * 8,
            static_cast<size_t>(std::ceil(static_cast<float>(dst.rows) / 8.0f)) * 8
        };

        // Run the kernel in async mode.
        kernel.args(
            cv::ocl::KernelArg::ReadOnlyNoSize(src),
            cv::Vec4i{
                safe_region.x, safe_region.y,
                safe_region.width, safe_region.height
            },
            cv::Vec2f{1,1},
            cv::ocl::KernelArg::WriteOnlyNoSize(dst),
            cv::Vec2i{dst.cols, dst.rows},
            cv::ocl::KernelArg::ReadOnlyNoSize(offsets)
        ).run_(2, global_work_size, local_work_size, false);

        // Create next kernel while the last one runs.
        kernel.create("easu_remap", yuv ? program_yuv : program_bgr);
        kernel_is_yuv = yuv;
    }

//---------------------------------------------------------------------------------------------------------------------

}