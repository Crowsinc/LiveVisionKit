//    *************************** LiveVisionKit ****************************
//    Copyright (C) 2022  Sebastian Di Marco (crowsinc.dev@gmail.com)
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.
// 	  **********************************************************************

#include "VideoFilter.hpp"

#include <opencv2/core/ocl.hpp>

namespace lvk
{
//---------------------------------------------------------------------------------------------------------------------

    VideoFilter::VideoFilter(const std::string filter_name)
        : m_Alias(filter_name + std::to_string(this->uid()))
    {}

//---------------------------------------------------------------------------------------------------------------------

    void VideoFilter::process(
        cv::VideoCapture& input_stream, 
        cv::VideoWriter& output_stream,
        const bool debug,
        const std::function<void(VideoFilter&, cv::UMat&)> callback
    )
    {
        LVK_ASSERT(input_stream.isOpened());
        LVK_ASSERT(output_stream.isOpened());
        
        cv::UMat frame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
        while (input_stream.read(frame))
        {
            process(frame, debug);
            
            if(!frame.empty())
                output_stream.write(frame);
            
            callback(*this, frame);
        }
    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoFilter::profile(cv::UMat& frame, Stopwatch& timer, const bool debug)
    {
        timer.start();

        process(frame, debug);
        cv::ocl::finish();
        
        timer.stop();
    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoFilter::profile(
        cv::VideoCapture& input_stream,
        cv::VideoWriter& output_stream,
        Stopwatch& timer,
        const bool debug,
        const std::function<void(VideoFilter&, cv::UMat&, Stopwatch&)> callback
    )
    {
        LVK_ASSERT(input_stream.isOpened());
        LVK_ASSERT(output_stream.isOpened());

        cv::UMat frame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
        while (input_stream.read(frame))
        {
            profile(frame, timer, debug);

            if (!frame.empty())
                output_stream.write(frame);

            callback(*this, frame, timer);
        }
    }
   
//---------------------------------------------------------------------------------------------------------------------

    const std::string& VideoFilter::alias() const
    {
        return m_Alias;
    }

//---------------------------------------------------------------------------------------------------------------------

}