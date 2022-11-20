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

    VideoFilter::VideoFilter(const std::string filter_name, const uint32_t timing_samples)
        : m_Alias(filter_name + std::to_string(this->uid())),
          m_FrameTimer(timing_samples)
    {}

//---------------------------------------------------------------------------------------------------------------------

    void VideoFilter::process(cv::UMat& frame, const bool debug)
    {
        //TODO: fix timing issues with OpenCL asynchrony
        m_FrameTimer.start();
        filter(frame, debug);
        cv::ocl::finish();
        m_FrameTimer.stop();
    }

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
        while(input_stream.read(frame))
        {
            process(frame, debug);
            
            if(!frame.empty())
                output_stream.write(frame);
            
            callback(*this, frame);
        }
    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoFilter::process(cv::UMat& frame, Logger& logger, const bool debug)
    {
        process(frame, debug);
        write_log(logger);
    }
   
//---------------------------------------------------------------------------------------------------------------------

    void VideoFilter::process(
        cv::VideoCapture& input_stream,
        cv::VideoWriter& output_stream,
        Logger& logger,
        const bool debug,
        const std::function<void(VideoFilter&, cv::UMat&, Logger&)> callback
    )
    {
        process(
            input_stream,
            output_stream,
            debug,
            [&](VideoFilter& filter, cv::UMat& frame)
            {
                write_log(logger);
                callback(filter, frame, logger);
            }
        );
    }

//---------------------------------------------------------------------------------------------------------------------

    Time VideoFilter::runtime() const
    {
        return m_FrameTimer.elapsed();
    }

//---------------------------------------------------------------------------------------------------------------------

    Time VideoFilter::runtime_average() const
    {
        return m_FrameTimer.average();
    }

//---------------------------------------------------------------------------------------------------------------------

    Time VideoFilter::runtime_deviation() const
    {
        return m_FrameTimer.deviation();
    }

//---------------------------------------------------------------------------------------------------------------------

    const std::string& VideoFilter::alias() const
    {
        return m_Alias;
    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoFilter::write_log(Logger& logger) {}

//---------------------------------------------------------------------------------------------------------------------

}