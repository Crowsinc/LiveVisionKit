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

#include "ConversionFilter.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

    ConversionFilter::ConversionFilter(const ConversionFilterSettings& settings)
        : VideoFilter("Conversion Filter")
    {
        this->configure(settings);
    }

//---------------------------------------------------------------------------------------------------------------------

    ConversionFilter::ConversionFilter(const cv::ColorConversionCodes conversion_code)
        : ConversionFilter(ConversionFilterSettings{.conversion_code = conversion_code})
    {}

//---------------------------------------------------------------------------------------------------------------------

    void ConversionFilter::process(
        const Frame& input,
        Frame& output,
        const bool debug
    )
    {
        LVK_ASSERT(!input.is_empty());

        output.timestamp = input.timestamp;

        cv::cvtColor(
            input.data,
            output.data,
            m_Settings.conversion_code,
            static_cast<int>(m_Settings.output_channels.value_or(0))
        );
    }


//---------------------------------------------------------------------------------------------------------------------

    void ConversionFilter::configure(const ConversionFilterSettings& settings)
    {
        m_Settings = settings;
    }

//---------------------------------------------------------------------------------------------------------------------

}