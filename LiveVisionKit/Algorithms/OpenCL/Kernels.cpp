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

cv::ocl::Kernel load_kernel(const char* kernel, const char* source, const char* flags)
{
    cv::String compilation_log;

    cv::ocl::ProgramSource program_source("", kernel, source, "");
    cv::ocl::Program program(program_source, flags, compilation_log);
    if(program.ptr() == nullptr)
    {
        // Perform custom assert with compilation error log.
        lvk::context::assert_handler(
            LVK_FILE,
            __func__,
            std::string("Failed to compile OpenCL kernel \'")
                + kernel + "\' with compilation log: \n\n" + compilation_log
        );
        return {};
    };
    return {kernel, program};
}

//---------------------------------------------------------------------------------------------------------------------

}