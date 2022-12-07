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
#include "Utility/Properties/Configurable.hpp"

#include <vector>

namespace lvk
{

    struct ConversionFilterSettings
    {
        std::vector<cv::ColorConversionCodes> conversion_chain;
    };

    class ConversionFilter final : public VideoFilter, public Configurable<ConversionFilterSettings>
    {
    public:

        explicit ConversionFilter(const ConversionFilterSettings& settings = {});

        explicit ConversionFilter(const cv::ColorConversionCodes conversion_code);

        explicit ConversionFilter(const std::vector<cv::ColorConversionCodes>& conversion_codes);

        using VideoFilter::process;
        void process(const Frame& input, Frame& output, const bool debug) override;

        void configure(const ConversionFilterSettings& settings) override;

    private:
        std::vector<cv::UMat> m_ConversionBuffers;
    };

}

