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

    Frame Frame::Wrap(cv::UMat& frame, const uint64_t timestamp)
    {
        Frame wrapped_frame;
        wrapped_frame.data = frame;
        wrapped_frame.timestamp = timestamp;
        return wrapped_frame;
    }

//---------------------------------------------------------------------------------------------------------------------

    Frame::Frame(const uint64_t timestamp)
        : data(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
          timestamp(timestamp)
    {}

//---------------------------------------------------------------------------------------------------------------------

    Frame::Frame(const cv::UMat& frame, const uint64_t timestamp)
        : data(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
          timestamp(timestamp)
    {
        frame.copyTo(data);
    }

//---------------------------------------------------------------------------------------------------------------------

    Frame::Frame(const cv::Size& size, const int type, const uint64_t timestamp)
        : data(size, type, cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
          timestamp(timestamp)
    {}

//---------------------------------------------------------------------------------------------------------------------

    Frame::Frame(const uint32_t width, const uint32_t height, const int type, const uint64_t timestamp)
        : data(
        static_cast<int>(height),
        static_cast<int>(width),
        type,
        cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY
    ),
          timestamp(timestamp)
    {}

    //---------------------------------------------------------------------------------------------------------------------

    Frame::Frame(const Frame& frame)
        : timestamp(frame.timestamp)
    {
        frame.data.copyTo(data);
    }

//---------------------------------------------------------------------------------------------------------------------

    Frame::Frame(Frame&& frame) noexcept
        : data(std::move(frame.data)),
          timestamp(frame.timestamp)
    {
        frame.data.release();
        frame.timestamp = 0;
    }

//---------------------------------------------------------------------------------------------------------------------

    Frame& Frame::operator=(Frame&& frame) noexcept
    {
        data = frame.data;
        timestamp = frame.timestamp;

        frame.data.release();
        frame.timestamp = 0;

        return *this;
    }

//---------------------------------------------------------------------------------------------------------------------

    void Frame::default_to(const cv::Size& size, const int type)
    {
        if(is_empty())
            allocate(size, type);
    }

//---------------------------------------------------------------------------------------------------------------------

    void Frame::default_to(const uint32_t width, const uint32_t height, const int type)
    {
        if(is_empty())
            allocate(width, height, type);
    }

//---------------------------------------------------------------------------------------------------------------------

    void Frame::allocate(const cv::Size& size, const int type)
    {
        data.release();
        data.create(
            size,
            type,
            cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY
        );
    }

//---------------------------------------------------------------------------------------------------------------------

    void Frame::allocate(const uint32_t width, const uint32_t height, const int type)
    {
        data.release();
        data.create(
            static_cast<int>(height),
            static_cast<int>(width),
            type,
            cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY
        );
    }

//---------------------------------------------------------------------------------------------------------------------

    void Frame::copy(const cv::UMat& src)
    {
        src.copyTo(data);
    }

//---------------------------------------------------------------------------------------------------------------------

    void Frame::copy(const Frame& src)
    {
        src.data.copyTo(data);
        timestamp = src.timestamp;
    }

//---------------------------------------------------------------------------------------------------------------------

    Frame Frame::clone() const
    {
        return Frame(data, timestamp);
    }

//---------------------------------------------------------------------------------------------------------------------

    uint32_t Frame::width() const
    {
        return data.cols;
    }

//---------------------------------------------------------------------------------------------------------------------

    uint32_t Frame::height() const
    {
        return data.rows;
    }

//---------------------------------------------------------------------------------------------------------------------

    cv::Size Frame::size() const
    {
        return data.size();
    }

//---------------------------------------------------------------------------------------------------------------------

    bool Frame::is_empty() const
    {
        return data.empty();
    }

//---------------------------------------------------------------------------------------------------------------------

    int Frame::type() const
    {
        return data.type();
    }

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