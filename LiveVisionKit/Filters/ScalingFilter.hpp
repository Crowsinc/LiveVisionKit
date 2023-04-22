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

#include "VideoFilter.hpp"
#include "Utility/Configurable.hpp"

namespace lvk
{

    struct ScalingFilterSettings
    {
        cv::Size output_size = {1920, 1080};
        float sharpness = 0.8f;
        bool yuv_input = true;
    };

    class ScalingFilter final : public VideoFilter, public Configurable<ScalingFilterSettings>
    {
    public:

        explicit ScalingFilter(const ScalingFilterSettings& settings = {});

        explicit ScalingFilter(const cv::Size& output_size, const float sharpness = 0.8f);

        void configure(const ScalingFilterSettings& settings) override;

    private:

        void filter(
            Frame&& input,
            Frame& output,
            Stopwatch& timer,
            const bool debug
        ) override;

    };

}

