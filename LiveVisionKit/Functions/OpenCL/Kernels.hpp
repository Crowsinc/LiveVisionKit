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

#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>

namespace lvk::ocl
{

    cv::ocl::Program load_program(const char* name, const char* source, const char* flags = "");

    // NOTE: The global and local groups and are arrays of length equal to buffer dimensions.
    void optimal_groups(const cv::UMat& buffer, size_t* local_groups, size_t* global_groups);

    // OpenCL Kernel Sources
    namespace src
    {
        inline const char* fsr_source =
            #include "FSR.cl"
        ;

        inline const char* drawing_source =
            #include "Drawing.cl"
        ;
    }
}
