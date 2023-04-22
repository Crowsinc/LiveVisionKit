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

#include "ScalingFilter.hpp"

#include "Functions/Image.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

    ScalingFilter::ScalingFilter(const ScalingFilterSettings& settings)
        : VideoFilter("Scaling Filter")
    {
        configure(settings);
    }

//---------------------------------------------------------------------------------------------------------------------

    ScalingFilter::ScalingFilter(const cv::Size& output_size, const float sharpness)
        : ScalingFilter(ScalingFilterSettings{.output_size=output_size,.sharpness=sharpness})
    {}

//---------------------------------------------------------------------------------------------------------------------

    void ScalingFilter::configure(const ScalingFilterSettings& settings)
    {
        LVK_ASSERT_01(settings.sharpness);
        LVK_ASSERT(settings.output_size.width > 0);
        LVK_ASSERT(settings.output_size.height > 0);

        m_Settings = settings;
    }

//---------------------------------------------------------------------------------------------------------------------

    void ScalingFilter::filter(
        Frame&& input,
        Frame& output,
        Stopwatch& timer,
        const bool debug
    )
    {
        LVK_ASSERT(!input.is_empty());

        lvk::upscale(input.data, output.data, m_Settings.output_size, m_Settings.yuv_input);
        lvk::sharpen(output.data, output.data, m_Settings.sharpness);
        output.timestamp = input.timestamp;
    }

//---------------------------------------------------------------------------------------------------------------------

}