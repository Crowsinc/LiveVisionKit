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

#include <thread>
#include <mutex>

namespace lvk
{
//---------------------------------------------------------------------------------------------------------------------

    VideoFilter::VideoFilter(const std::string& filter_name)
        : m_Alias(filter_name + " (" + std::to_string(this->uid()) + ")"),
          m_FrameTimer(30)
    {}

//---------------------------------------------------------------------------------------------------------------------

    void VideoFilter::process(
        const Frame& input,
        Frame& output,
        const bool debug
    )
    {
        filter(input, output, m_FrameTimer, debug);
    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoFilter::render(
        const Frame& input,
        bool debug
    )
    {
        filter(input, m_FrameBuffer, m_FrameTimer, debug);
        cv::imshow(alias(), m_FrameBuffer.data);
    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoFilter::process(
        cv::VideoCapture& input_stream, 
        cv::VideoWriter& output_stream,
        const bool debug,
        const std::function<bool(VideoFilter&, Frame&)>& callback
    )
    {
        LVK_ASSERT(input_stream.isOpened());
        LVK_ASSERT(output_stream.isOpened());

        Frame input_frame, output_frame;
        while (input_stream.read(input_frame.data))
        {
            // NOTE: not all input streams support timestamps (webcam etc.), such
            // stream will likely default to a timestamp of zero for all frames. 
            const auto stream_position = std::max<double>(0.0, input_stream.get(cv::CAP_PROP_POS_MSEC));
            input_frame.timestamp = static_cast<uint64_t>(Time::Milliseconds(stream_position).nanoseconds());
            
            process(input_frame, output_frame, debug);

            if(!output_frame.is_empty())
                output_stream.write(output_frame.data);
            
            if(callback(*this, output_frame))
                break;
        }
    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoFilter::render(
        cv::VideoCapture& input_stream,
        const bool debug,
        const std::function<bool(VideoFilter&, Frame&)>& callback
    )
    {
        LVK_ASSERT(input_stream.isOpened());

        Frame input_frame, output_frame;
        while (input_stream.read(input_frame.data))
        {
            // NOTE: not all input streams support timestamps (webcam etc.), such
            // stream will likely default to a timestamp of zero for all frames.
            const auto stream_position = std::max<double>(0.0, input_stream.get(cv::CAP_PROP_POS_MSEC));
            input_frame.timestamp = static_cast<uint64_t>(Time::Milliseconds(stream_position).nanoseconds());

            process(input_frame, output_frame, debug);
            cv::imshow(alias(), output_frame.data);

            if(callback(*this, output_frame))
                break;
        }
    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoFilter::set_timing_samples(const uint32_t samples)
    {
        m_FrameTimer = Stopwatch(samples);
    }

//---------------------------------------------------------------------------------------------------------------------

    const Stopwatch& VideoFilter::timings() const
    {
        return m_FrameTimer;
    }

//---------------------------------------------------------------------------------------------------------------------

    const std::string& VideoFilter::alias() const
    {
        return m_Alias;
    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoFilter::filter(
        const Frame& input,
        Frame& output,
        Stopwatch& timer,
        const bool debug
    )
    {
        // Default filter is simply an identity operation
        output.copy(input);
    }

//---------------------------------------------------------------------------------------------------------------------

}