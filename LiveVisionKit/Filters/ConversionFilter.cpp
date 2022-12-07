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

    ConversionFilter::ConversionFilter(const cv::ColorConversionCodes conversion_code)
        : ConversionFilter(std::vector<cv::ColorConversionCodes>(1, conversion_code))
    {}

//---------------------------------------------------------------------------------------------------------------------

    ConversionFilter::ConversionFilter(const std::vector<cv::ColorConversionCodes>& conversion_codes)
        : VideoFilter("Conversion Filter")
    {
        this->configure(conversion_codes);
    }

//---------------------------------------------------------------------------------------------------------------------

    void ConversionFilter::process(
        const Frame& input,
        Frame& output,
        const bool debug
    )
    {
        // If there is no conversion, then this is simply identity
        if(m_Settings.empty())
            VideoFilter::process(input, output, false);

        // If there is only one conversion, then convert directly to the output
        if(m_Settings.size() == 1)
            cv::cvtColor(input.data, output.data, m_Settings.front());


        cv::UMat src_buffer = input.data;
        for(size_t i = 0; i < m_IntermediateBuffers.size(); i++)
        {
            cv::cvtColor(src_buffer, m_IntermediateBuffers[i], m_Settings[i]);
            src_buffer = m_IntermediateBuffers[i];
        }
        cv::cvtColor(m_IntermediateBuffers.back(), output.data, m_Settings.back());
    }

//---------------------------------------------------------------------------------------------------------------------

    void ConversionFilter::configure(const std::vector<cv::ColorConversionCodes>& conversion_codes)
    {
        m_Settings = conversion_codes;

        if(conversion_codes.size() > 1)
        {
            // Make sure we have the exact amount of intermediate buffers
            // required to perform all the conversions. Note that the final
            // conversion is done straight to the output.

            m_IntermediateBuffers.resize(conversion_codes.size() - 1);
        }
        else m_IntermediateBuffers.clear();
    }

//---------------------------------------------------------------------------------------------------------------------

}