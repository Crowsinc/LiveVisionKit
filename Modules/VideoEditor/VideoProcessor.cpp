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

#include "VideoProcessor.hpp"

#include <type_traits>
#include <utility>

namespace clt
{
//---------------------------------------------------------------------------------------------------------------------

    constexpr size_t FILTER_TIMING_SAMPLES = 300;
    constexpr const char* RENDER_WINDOW_NAME = "LVK Output";

//---------------------------------------------------------------------------------------------------------------------

    VideoProcessor::VideoProcessor(VideoIOConfiguration configuration)
        : m_Configuration(std::move(configuration))
    {}

//---------------------------------------------------------------------------------------------------------------------

    std::optional<std::string> VideoProcessor::initialize_configuration()
    {
        // Open input stream
        std::optional<std::string> input_error;
        std::visit([&, this](auto&& source){
            using source_type = std::decay_t<decltype(source)>;

            if constexpr(std::is_same_v<source_type, std::filesystem::path>)
            {
                std::vector<int> properties = {
                    cv::CAP_PROP_HW_ACCELERATION, 1,
                    cv::CAP_PROP_HW_ACCELERATION_USE_OPENCL, 1
                };

                m_DeviceCapture = false;
                m_InputStream = cv::VideoCapture(source.string(), cv::CAP_FFMPEG, properties);
                if(!m_InputStream.isOpened())
                    input_error = cv::format("Failed to open the input video \'%s\'", source.string().c_str());
            }
            else if constexpr(std::is_same_v<source_type, uint32_t>)
            {
                m_DeviceCapture = true;
                m_InputStream = cv::VideoCapture(source);
                if(!m_InputStream.isOpened())
                    input_error = cv::format("Failed to capture device \'%u\'", source);
            }
            else input_error = "No input source was specified!";
        },
        m_Configuration.input_source);

        if(input_error.has_value())
            return input_error;

        // Configure the filter
        m_Processor.reconfigure([&](lvk::CompositeFilterSettings& settings){
            // Add BGR to YUV conversion as LVK filters run on a YUV standard
            settings.filter_chain.emplace_back(
                new lvk::ConversionFilter(lvk::ConversionFilterSettings{.conversion_code=cv::COLOR_BGR2YUV})
            );

            for(auto& filter : m_Configuration.filter_chain)
            {
                filter->set_timing_samples(FILTER_TIMING_SAMPLES);
                settings.filter_chain.push_back(filter);
            }

            // Convert back to BGR OpenCV standard for output
            settings.filter_chain.emplace_back(
                new lvk::ConversionFilter(lvk::ConversionFilterSettings{.conversion_code=cv::COLOR_YUV2BGR})
            );
        });

        // Load data logger
        if(m_Configuration.log_target.has_value())
        {
            m_DataLogStream.open(*m_Configuration.log_target);
            if(!m_DataLogStream.good())
                return "Failed to open data logging stream";

            m_DataLogger.emplace(m_DataLogStream);
        }

        return std::nullopt;
    }

//---------------------------------------------------------------------------------------------------------------------

    std::optional<std::string> VideoProcessor::initialize_output_stream(const cv::Size frame_size)
    {
        if(!m_Configuration.output_target.has_value())
            return "Could not create output stream, no target was specified";

        try {
            std::vector<int> properties = {
                cv::VideoWriterProperties::VIDEOWRITER_PROP_HW_ACCELERATION, 1,
                cv::VideoWriterProperties::VIDEOWRITER_PROP_HW_ACCELERATION_USE_OPENCL, 1
            };

            m_OutputStream = cv::VideoWriter(
                m_Configuration.output_target->string(),
                cv::CAP_FFMPEG,
                m_Configuration.output_codec.value_or(
                    static_cast<int>(m_InputStream.get(cv::CAP_PROP_FOURCC))
                ),
                m_Configuration.output_framerate.value_or(
                    std::max(m_InputStream.get(cv::CAP_PROP_FPS), 1.0)
                ),
                frame_size,
                properties
            );
        }
        catch(std::exception& e)
        {
            return cv::format(
                "Failed to create output stream with error \'%s\'",
                e.what()
            );
        }

        // If stream is still not opened, then creation failed
        if(!m_OutputStream.isOpened())
        {
            return cv::format(
                "Failed to create an output stream at \'%s\'",
                m_Configuration.output_target->string().c_str()
            );
        }

        return std::nullopt;
    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoProcessor::stop()
    {
        m_Terminate = true;
    }

//---------------------------------------------------------------------------------------------------------------------

    std::optional<std::string> VideoProcessor::run()
    {
        std::optional<std::string> runtime_error;

        runtime_error = initialize_configuration();
        if(runtime_error.has_value())
            return runtime_error;

        // Create output window, making sure its resizable
        if(m_Configuration.render_output)
            cv::namedWindow(RENDER_WINDOW_NAME, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);

        m_FrameTimer.start();
        m_ProcessTimer.start();
        lvk::Time last_update_time;

        // Run the processor filter
        m_Terminate = false;
        m_Processor.process(
            m_InputStream,
            [&, this](lvk::VideoFilter& filter, lvk::Frame& frame)
            {

                // Write output
                if(m_Configuration.output_target.has_value())
                {
                    // Lazily initialize the output stream on first output frame
                    if(!m_OutputStream.isOpened())
                    {
                        runtime_error = initialize_output_stream(frame.size());
                        if(runtime_error.has_value())
                            return true;
                    }

                    m_OutputStream.write(frame.data);
                }

                // Display output
                if(m_Configuration.render_output)
                {
                    cv::imshow(RENDER_WINDOW_NAME, frame.data);

                    // Close display if escape is pressed, also note that
                    // the poll event is required to update the window.
                    if(const auto key = cv::pollKey(); key == 27)
                    {
                        m_Configuration.render_output = false;
                        cv::destroyAllWindows();

                        // If the input is a device capture or there is no output path, then
                        // we consider the display to the output. So closing the window should
                        // also terminate the processing. This is so that we can decide when to
                        // end indefinite device capture streams, and to avoid accidentally
                        // leaving the processor running in the background indefinitely.
                        return m_DeviceCapture || !m_Configuration.output_target.has_value();
                    }
                }

                // Update the frame timer
                if(m_Configuration.render_output && m_Configuration.render_period.has_value())
                {
                    // If we are displaying the output at a fixed frequency,
                    // then we need to wait to match the user's timestep here.
                    m_FrameTimer.tick(*m_Configuration.render_period);
                }
                else m_FrameTimer.tick();

                // Run all update procedures (logging etc.)
                const auto elapsed_time = m_ProcessTimer.elapsed();
                if(last_update_time.is_zero() || elapsed_time > last_update_time + m_Configuration.update_period)
                {
                    last_update_time = elapsed_time;
                    write_to_loggers();
                }

                return m_Terminate;
            },
            m_Configuration.debug_mode
        );

        // Run loggers one last time to ensure we have the latest statistics displayed.
        write_to_loggers();

        return runtime_error;
    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoProcessor::write_to_loggers()
    {
        m_ConsoleLogger.clear();

        print_progress();
        if(m_Configuration.print_timings)
            print_filter_timings();

        if(m_DataLogger.has_value())
            log_timing_data();
    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoProcessor::print_progress()
    {
        // NOTE: The frame count is not valid for device capture streams
        double frame_count = m_InputStream.get(cv::CAP_PROP_FRAME_COUNT);
        double frame_number = m_InputStream.get(cv::CAP_PROP_POS_FRAMES);

        // Input Stream Info
        m_ConsoleLogger << "Processing target: ";
        if(!m_DeviceCapture)
        {
            m_ConsoleLogger << std::get<std::filesystem::path>(m_Configuration.input_source).string()
                            << "  " << make_progress_bar(40, frame_number / frame_count)
                            << ConsoleLogger::Next;
        }
        else m_ConsoleLogger << "Device Capture" << ConsoleLogger::Next;

        // Print Elapsed time
        m_ConsoleLogger << "   Elapsed: " << m_ProcessTimer.elapsed().hms();
        if(!m_DeviceCapture)
        {
            lvk::Time est_remaining_time = lvk::Time::Seconds(
                std::ceil((frame_count - frame_number) / m_FrameTimer.average().frequency())
            );
            m_ConsoleLogger << " (est. " << est_remaining_time.hms() << " remaining)";
        }
        m_ConsoleLogger << ConsoleLogger::Next;

        // Print current frame input
        // NOTE: CAP_PROP_POS_FRAMES may not be supported by the VideoCapture backend, leading
        // to an invalid frame number. If we suspect this is the case, switch over to using the
        // frame timers tick count, which counts all but any empty output frames.
        m_ConsoleLogger << "   Frame: "
                        << ((frame_number <= 0) ? m_FrameTimer.tick_count() : static_cast<uint64_t>(frame_number))
                        << ConsoleLogger::Next;

        // Print current FPS
        m_ConsoleLogger << "   FPS: "
                        << std::fixed << std::setprecision(0) << m_FrameTimer.average().frequency()
                        << ConsoleLogger::Next;
    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoProcessor::print_filter_timings()
    {
        // Print timing data for each of the filters
        m_ConsoleLogger << std::setprecision(2);
        m_ConsoleLogger << ConsoleLogger::Next << "Filters: " << ConsoleLogger::Next;
        for(size_t i = 0; i < m_Processor.filter_count(); i++)
        {
            auto filter = m_Processor.filters(i);
            auto average_timing = filter->timings().average();

            m_ConsoleLogger << std::to_string(i) <<  ".   "
                            << filter->alias()
                            << "\t" << average_timing.milliseconds() << "ms"
                            << " +/- " << filter->timings().deviation().milliseconds() << "ms"
                            << "   (" << static_cast<uint64_t>(average_timing.frequency()) << "FPS)"
                            << ConsoleLogger::Next;
        }
    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoProcessor::log_timing_data()
    {
        LVK_ASSERT(m_DataLogger.has_value());

        lvk::CSVLogger& logger = *m_DataLogger;

        // On first log, add all the headers as filter names
        if(!logger.has_started())
        {
            // Format is..
            // 1. Output Frame Number
            // 2. Processor frametime
            // 3. All filter frametimes
            // 4. Processor deviation
            // 5. All filter deviations

            logger << "Output Frame";

            logger << "Processor Frametime (ms)";
            for(auto& filter : m_Processor.filters())
                logger << (filter->alias() + " Frametime (ms)");

            // Then log all the frame deviation times
            logger << "Processor Deviation (ms)";
            for(auto& filter : m_Processor.filters())
                logger << (filter->alias() + " Deviation (ms)");

            logger.next();
        }

        // write frame number
        logger << m_FrameTimer.tick_count();

        // write all frametimes
        logger << m_FrameTimer.average().milliseconds();
        for(auto& filter : m_Processor.filters())
            logger << filter->timings().average().milliseconds();

        // write all frame deviation times
        logger << m_FrameTimer.deviation().milliseconds();
        for(auto& filter : m_Processor.filters())
            logger << filter->timings().deviation().milliseconds();

        logger.next();
    }

//---------------------------------------------------------------------------------------------------------------------

    std::string VideoProcessor::make_progress_bar(const uint32_t length, const double progress)
    {
        LVK_ASSERT_01(progress);

        const auto position = static_cast<uint32_t>(progress * static_cast<double>(length));

        std::string bar;
        bar.reserve(length + 10);

        bar.push_back('[');
        for(uint32_t i = 0; i < length; i++)
            bar.push_back(i < position ? '=' : ' ');
        bar += "| " + cv::format("%0.1f%%", 100.0 * progress) + ']';

        return bar;
    }

//---------------------------------------------------------------------------------------------------------------------

}