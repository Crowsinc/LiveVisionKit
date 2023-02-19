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

#include <condition_variable>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>

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
        filter(Frame(input), output, m_FrameTimer, debug);
    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoFilter::process(
        Frame&& input,
        Frame& output,
        const bool debug
    )
    {
        filter(std::move(input), output, m_FrameTimer, debug);
    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoFilter::process(
        cv::VideoCapture& input_stream,
        const std::function<bool(VideoFilter&, Frame&)>& callback,
        const bool debug
    )
    {
        LVK_ASSERT(input_stream.isOpened());

        const size_t max_buffer_frames = 15;

        std::mutex input_mutex, output_mutex;
        std::queue<Frame> input_queue, output_queue;

        std::condition_variable input_consume_flag, output_consume_flag;
        std::condition_variable input_available_flag, output_available_flag;
        std::atomic<bool> input_finished = false, filter_finished = false, terminate_input = false;

        // Input Processor
        // This reads frames from the input stream and passes them off for filtering.
        auto input_thread = std::thread([&](){
            Frame read_frame;
            while(input_stream.read(read_frame.data) && !terminate_input)
            {
                // Set frame timestamp if supported, otherwise set it to zero.
                const auto stream_position = std::max(0.0, input_stream.get(cv::CAP_PROP_POS_MSEC));
                read_frame.timestamp = static_cast<uint64_t>(Time::Milliseconds(stream_position).nanoseconds());

                // Push new frame onto the input queue
                {
                    std::unique_lock<std::mutex> queue_lock(input_mutex);

                    // If the input queue is saturated, wait until a frame is consumed
                    while(input_queue.size() >= max_buffer_frames)
                        input_consume_flag.wait(queue_lock);

                    input_queue.push(std::move(read_frame));
                    if(input_queue.size() == 1)
                        input_available_flag.notify_one();
                }
            }
            input_finished = true;
            input_available_flag.notify_one();
        });


        // Filter Processor
        // This grabs frames delivered by the input processor, filters them, and passes them off for output.
        auto filter_thread = std::thread([&](){
            Frame input_frame, filtered_frame;
            while(true)
            {
                // Pop a frame from the input queue
                {
                    std::unique_lock<std::mutex> queue_lock(input_mutex);
                    while(input_queue.empty())
                    {
                        // If there are no new frames incoming, then we have filtered everything
                        if(input_finished)
                        {
                            filter_finished = true;
                            output_available_flag.notify_one();
                            return;
                        }

                        input_available_flag.wait(queue_lock);
                    }

                    input_frame = std::move(input_queue.front());
                    input_queue.pop();

                    input_consume_flag.notify_one();
                }

                // Process the frame
                process(std::move(input_frame), filtered_frame, debug);
                if(filtered_frame.is_empty())
                    continue;

                // Push processed frame onto the output queue
                {
                    std::unique_lock<std::mutex> queue_lock(output_mutex);

                    // If the output queue is saturated, wait until a frame is consumed
                    while(output_queue.size() >= max_buffer_frames)
                        output_consume_flag.wait(queue_lock);

                    output_queue.push(std::move(filtered_frame));
                    if(output_queue.size() == 1)
                        output_available_flag.notify_one();
                }
            }
        });


        // Output Processor
        // This grabs filtered frames delivered by the filter processor and sends them to the user callback.
        Frame output_frame;
        while(true)
        {
            // Pop next frame from the output queue
            {
                std::unique_lock<std::mutex> queue_lock(output_mutex);
                while(output_queue.empty())
                {
                    // If there are no new frames incoming, then we have finished processing
                    if(filter_finished)
                    {
                        input_thread.join();
                        filter_thread.join();
                        return;
                    }

                    output_available_flag.wait(queue_lock);
                }

                output_frame = std::move(output_queue.front());
                output_queue.pop();

                output_consume_flag.notify_one();
            }

            // Send frame to the output
            if(callback(*this, output_frame))
            {
                // User called for the processing to be terminated.

                // To avoid complicating the multi-threading process we will simply
                // terminate the input, then starve out the input and output queues
                // to emulate reaching the end of the input stream.

                // Terminate input and starve out the filter thread.
                terminate_input = true;
                {
                    std::unique_lock<std::mutex> queue_lock(input_mutex);
                    while(!input_queue.empty()) input_queue.pop();
                    input_consume_flag.notify_one();
                }
                input_thread.join();

                // Make space in the output queue to avoid saturating the filter thread.
                {
                    std::unique_lock<std::mutex> queue_lock(output_mutex);
                    while(!output_queue.empty()) output_queue.pop();
                    output_consume_flag.notify_one();
                }
                filter_thread.join();
                return;
            }
        }

    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoFilter::render(
        const Frame& input,
        bool debug
    )
    {
        process(input, m_FrameBuffer, debug);
        cv::imshow(alias(), m_FrameBuffer.data);
    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoFilter::render(
        cv::VideoCapture& input_stream,
        const uint32_t target_fps,
        const bool debug
    )
    {
        LVK_ASSERT(input_stream.isOpened());

        TickTimer frame_timer;
        Time target_frametime = Time::Timestep(
            (target_fps == 0) ? input_stream.get(cv::CAP_PROP_FPS) : target_fps
        );

        frame_timer.start();
        process(input_stream, [&](VideoFilter& filter, Frame& frame){
            cv::imshow(filter.alias(), frame.data);

            frame_timer.tick(target_frametime);

            return cv::pollKey() == 27; // Esc
        }, debug);
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
        Frame&& input,
        Frame& output,
        Stopwatch& timer,
        const bool debug
    )
    {
        // Default filter is simply an identity operation
        output = std::move(input);
    }

//---------------------------------------------------------------------------------------------------------------------

}