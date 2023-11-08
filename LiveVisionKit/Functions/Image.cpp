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

    void remap(const VideoFrame& src, VideoFrame& dst, const cv::UMat& offset_map, const cv::Scalar& background)
    {
        LVK_ASSERT(offset_map.type() == CV_32FC2);
        LVK_ASSERT(src.cols > 0 && src.rows > 0);
        LVK_ASSERT(src.type() == CV_8UC3);
        LVK_ASSERT(!offset_map.empty());
        LVK_ASSERT(!src.empty());

        const bool yuv = src.format == VideoFrame::YUV;

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

        // Allocate the output based on the size of the offset map. This allows
        // an ROI of the source to be remapped and scaling operations to occur.
        dst.create(offset_map.size(), CV_8UC3);

        // We need to account for the ROI offset in the map
        // when we create the output coordinates in the kernel.
        cv::Size map_size; cv::Point dst_offset;
        offset_map.locateROI(map_size, dst_offset);

        // Find optimal work sizes for the 2D dst buffer.
        size_t global_work_size[3], local_work_size[3];
        ocl::optimal_groups(dst, global_work_size, local_work_size);

        // Run the kernel in async mode.
        kernel.args(
            cv::ocl::KernelArg::ReadOnly(src),
            cv::ocl::KernelArg::WriteOnlyNoSize(dst),
            cv::Vec4i{dst_offset.x, dst_offset.y, dst.cols, dst.rows},
            cv::ocl::KernelArg::ReadOnlyNoSize(offset_map),
            cv::Vec4b(
                static_cast<uint8_t>(background[0]),
                static_cast<uint8_t>(background[1]),
                static_cast<uint8_t>(background[2]),
                0 // NOTE: 4th component is unused
            )
        ).run_(2, global_work_size, local_work_size, false);

        // Create next kernel while the last one runs.
        kernel.create("easu_remap", yuv ? program_yuv : program_bgr);
        kernel_is_yuv = yuv;
    }

//---------------------------------------------------------------------------------------------------------------------

    void remap(
        const VideoFrame& src,
        VideoFrame& dst,
        const cv::Mat& homography,
        const cv::Scalar& background,
        const bool inverted
    )
    {
        LVK_ASSERT(homography.cols == 3 && homography.rows == 3);
        LVK_ASSERT(homography.type() == CV_64FC1);
        LVK_ASSERT(src.cols > 0 && src.rows > 0);
        LVK_ASSERT(src.type() == CV_8UC3);
        LVK_ASSERT(!homography.empty());
        LVK_ASSERT(!src.empty());

        const bool yuv = src.format == VideoFrame::YUV;

        // FSR program has yuv and bgr versions for different luma calculations.
        static auto program_yuv = ocl::load_program("fsr", ocl::src::fsr_source, "-D YUV_INPUT");
        static auto program_bgr = ocl::load_program("fsr", ocl::src::fsr_source);
        LVK_ASSERT(!program_yuv.empty() && !program_bgr.empty());

        // Create FSR EASU kernel
        thread_local cv::ocl::Kernel kernel;
        thread_local bool kernel_is_yuv = yuv;
        if(kernel.empty() || kernel_is_yuv != yuv)
        {
            kernel.create("easu_remap_homography", yuv ? program_yuv : program_bgr);
        }

        // Allocate the output based on the input size.
        dst.create(src.size(), CV_8UC3);

        // We need to account for the ROI offset in the dst
        // when we create the output coordinates in the kernel.
        cv::Size map_size; cv::Point dst_offset(0,0);
        dst.locateROI(map_size, dst_offset);

        // Find optimal work sizes for the 2D dst buffer.
        size_t global_work_size[3], local_work_size[3];
        ocl::optimal_groups(dst, global_work_size, local_work_size);

        // Invert homography if it isn't already.
        cv::Mat t;
        if(inverted) t = homography;
        else t = homography.inv();

        // Run the kernel in async mode.
        kernel.args(
                cv::ocl::KernelArg::ReadOnly(src),
                cv::ocl::KernelArg::WriteOnlyNoSize(dst),
                cv::Vec4i{dst_offset.x, dst_offset.y, dst.cols, dst.rows},
                cv::Vec4f(t.at<double>(0,0), t.at<double>(0,1), t.at<double>(0,2), 0),
                cv::Vec4f(t.at<double>(1,0), t.at<double>(1,1), t.at<double>(1,2), 0),
                cv::Vec4f(t.at<double>(2,0), t.at<double>(2,1), t.at<double>(2,2), 0),
                cv::Vec4b(
                        static_cast<uint8_t>(background[0]),
                        static_cast<uint8_t>(background[1]),
                        static_cast<uint8_t>(background[2]),
                        0 // NOTE: 4th component is unused
                )
        ).run_(2, global_work_size, local_work_size, false);

        // Create next kernel while the last one runs.
        kernel.create("easu_remap_homography", yuv ? program_yuv : program_bgr);
        kernel_is_yuv = yuv;
    }

//---------------------------------------------------------------------------------------------------------------------

    void upscale(const cv::UMat& src, cv::UMat& dst, const cv::Size& size, const bool yuv)
    {
        LVK_ASSERT(size.width >= src.cols && size.height >= src.rows);
        LVK_ASSERT(src.cols > 0 && src.rows > 0);
        LVK_ASSERT(src.type() == CV_8UC3);
        LVK_ASSERT(!src.empty());

        if(size == src.size())
        {
            src.copyTo(dst);
            return;
        }

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

        // Find optimal work sizes for the 2D dst buffer.
        size_t global_work_size[3], local_work_size[3];
        ocl::optimal_groups(dst, global_work_size, local_work_size);

        // Run the kernel in async mode.
        kernel.args(
            cv::ocl::KernelArg::ReadOnly(src),
            cv::ocl::KernelArg::WriteOnly(dst),
            cv::Vec2f{
                static_cast<float>(src.cols) / static_cast<float>(dst.cols),
                static_cast<float>(src.rows) / static_cast<float>(dst.rows)
            }
        ).run_(2, global_work_size, local_work_size, false);

        // Create next kernel while the last one runs.
        kernel.create("easu_scale", yuv ? program_yuv : program_bgr);
        kernel_is_yuv = yuv;
    }

//---------------------------------------------------------------------------------------------------------------------

    void sharpen(const cv::UMat& src, cv::UMat& dst, const float sharpness)
    {
        LVK_ASSERT(src.cols > 0 && src.rows > 0);
        LVK_ASSERT(src.type() == CV_8UC3);
        LVK_ASSERT_01(sharpness);
        LVK_ASSERT(!src.empty());

        // Create FSR RCAS kernel
        static auto program = ocl::load_program("fsr", ocl::src::fsr_source);
        thread_local cv::ocl::Kernel kernel("rcas", program);
        LVK_ASSERT(!program.empty() && !kernel.empty());

        // Allocate the output.
        dst.create(src.size(), CV_8UC3);

        // Find optimal work sizes for the 2D dst buffer.
        size_t global_work_size[3], local_work_size[3];
        ocl::optimal_groups(dst, global_work_size, local_work_size);

        // Run the kernel in async mode.
        kernel.args(
            cv::ocl::KernelArg::ReadOnly(src),
            cv::ocl::KernelArg::WriteOnlyNoSize(dst),
            std::exp2(-2.0f * (1.0f - sharpness))
        ).run_(2, global_work_size, local_work_size, false);

        // Create next kernel while the last one runs.
        kernel.create("rcas", program);
    }

//---------------------------------------------------------------------------------------------------------------------

}