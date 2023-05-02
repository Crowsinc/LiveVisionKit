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

#include "Kernels.hpp"

#include "Directives.hpp"

namespace lvk::ocl
{

//---------------------------------------------------------------------------------------------------------------------

cv::ocl::Program load_program(const char* name, const char* source, const char* flags)
{
    cv::String compilation_log;

    cv::ocl::ProgramSource program_source(name, name, source, "");
    cv::ocl::Program program(program_source, flags, compilation_log);
    if(program.ptr() == nullptr)
    {
        // Perform custom assert with compilation error log.
        lvk::context::assert_handler(
            LVK_FILE,
            __func__,
            std::string("Failed to compile OpenCL program \'")
                + name + "\' with compilation log: \n\n" + compilation_log
        );
        return {};
    };
    return std::move(program);
}

//---------------------------------------------------------------------------------------------------------------------

    void optimal_groups(const cv::UMat& buffer, size_t global_groups[3], size_t local_groups[3])
    {
        // Reset all group sizings to identity.
        local_groups[0] = 1; local_groups[1] = 1; local_groups[2] = 1;
        global_groups[0] = 1; global_groups[1] = 1; global_groups[2] = 1;

        // Figure out optimal and compatible 2D local and global work sizes for a kernel.
        // Note that this is just based on rule of thumbs, rather than concrete optimality.
        if(buffer.dims == 1 || buffer.cols == 1)
        {
            // 1D buffers: default to a work group of 64x1 threads.
            local_groups[0] = 64;
            global_groups[0] = static_cast<size_t>(std::ceil(static_cast<float>(buffer.rows) / 64.0f)) * 64;
        }
        else if(buffer.dims == 2)
        {
            // 2D buffers: default to a work group of 8x8 threads.
            local_groups[0] = 8; local_groups[1] = 8;
            global_groups[0] = static_cast<size_t>(std::ceil(static_cast<float>(buffer.cols) / 8.0f)) * 8;
            global_groups[1] = static_cast<size_t>(std::ceil(static_cast<float>(buffer.rows) / 8.0f)) * 8;
        }
        else LVK_ASSERT(false && "Buffer dimensions are not supported");
    }

//---------------------------------------------------------------------------------------------------------------------

}