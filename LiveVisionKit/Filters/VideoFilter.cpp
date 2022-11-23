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
        const std::function<void(VideoFilter&, Frame&)> callback
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

            if(!output_frame.empty())
                output_stream.write(output_frame.data);
            
            callback(*this, output_frame);
        }
    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoFilter::profile(
        const Frame& input,
        Frame& output,
        Stopwatch& timer,
        const bool debug
    )
    {
        timer.start();

        process(input, output, debug);
        cv::ocl::finish();
        
        timer.stop();
    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoFilter::profile(
        cv::VideoCapture& input_stream,
        cv::VideoWriter& output_stream,
        Stopwatch& timer,
        const bool debug,
        const std::function<void(VideoFilter&, Frame&, Stopwatch&)> callback
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

            profile(input_frame, output_frame, timer, debug);

            if (!output_frame.empty())
                output_stream.write(output_frame.data);

            callback(*this, output_frame, timer);
        }
    }
   
//---------------------------------------------------------------------------------------------------------------------

    const std::string& VideoFilter::alias() const
    {
        return m_Alias;
    }

//---------------------------------------------------------------------------------------------------------------------

    Frame&& Frame::Allocate(
        const cv::Size& size,
        const int type,
        const uint32_t timestamp
    )
    {
        Frame frame;
        frame.timestamp = timestamp;
        frame.try_allocate(size, type);
        return std::move(frame);
    }

//---------------------------------------------------------------------------------------------------------------------

    Frame&& Frame::Allocate(
        const uint32_t width,
        const uint32_t height,
        const int type,
        const uint32_t timestamp
    )
    {
        Frame frame;
        frame.timestamp = timestamp;
        frame.try_allocate(width, height, type);
        return std::move(frame);
    }

//---------------------------------------------------------------------------------------------------------------------

    Frame&& Frame::Wrap(cv::UMat& frame, const uint32_t timestamp)
    {
        Frame wrapped_frame;
        wrapped_frame.data = frame;
        wrapped_frame.timestamp = timestamp;
        return std::move(wrapped_frame);
    }

//---------------------------------------------------------------------------------------------------------------------

    Frame&& Frame::Copy(const cv::UMat& frame, const uint32_t timestamp = 0)
    {
        Frame copy_frame;
        copy_frame.copy_from(frame);
        copy_frame.timestamp = timestamp;
        return std::move(copy_frame);
    }

//---------------------------------------------------------------------------------------------------------------------

    Frame::Frame()
        : data(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
          timestamp(0)
    {}

//---------------------------------------------------------------------------------------------------------------------

    Frame::Frame(Frame&& frame) noexcept
        : data(frame.data),
          timestamp(frame.timestamp)
    {
        frame.data.release();
        frame.timestamp = 0;
    }

//---------------------------------------------------------------------------------------------------------------------

    void Frame::operator=(Frame&& frame) noexcept
    {
        data = frame.data;
        timestamp = frame.timestamp;

        frame.data.release();
        frame.timestamp = 0;
    }

//---------------------------------------------------------------------------------------------------------------------

    void Frame::try_allocate(const cv::Size& size, const int type)
    {
        data.create(
            size,
            type,
            cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY
        );
    }

//---------------------------------------------------------------------------------------------------------------------

    void Frame::try_allocate(const uint32_t width, const uint32_t height, const int type)
    {
        data.create(
            height,
            width,
            type,
            cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY
        );
    }

//---------------------------------------------------------------------------------------------------------------------

    void Frame::copy_from(const cv::UMat& src)
    {
        src.copyTo(data);
    }

//---------------------------------------------------------------------------------------------------------------------

    void Frame::copy_from(const Frame& src)
    {
        src.data.copyTo(data);
    }

//---------------------------------------------------------------------------------------------------------------------

    Frame&& Frame::copy() const
    {
        return Frame::Copy(data, timestamp);
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

    bool Frame::empty() const
    {
        return data.empty();
    }

//---------------------------------------------------------------------------------------------------------------------

    int Frame::type() const
    {
        return data.type();
    }

//---------------------------------------------------------------------------------------------------------------------

}