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
#include <optional>

#include "VideoFilter.hpp"
#include "Utility/Configurable.hpp"

namespace lvk
{

    struct ConversionFilterSettings
    {
        cv::ColorConversionCodes conversion_code = cv::COLOR_BGR2YUV;
        std::optional<size_t> output_channels;
    };

    class ConversionFilter final : public VideoFilter, public Configurable<ConversionFilterSettings>
    {
    public:

        explicit ConversionFilter(const ConversionFilterSettings& settings = {});

        explicit ConversionFilter(const cv::ColorConversionCodes conversion_code);

        void configure(const ConversionFilterSettings& settings) override;

    private:

        void filter(VideoFrame&& input, VideoFrame& output) override;

    };

}

