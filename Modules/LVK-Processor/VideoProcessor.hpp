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

#include <LiveVisionKit.hpp>
#include <fstream>

#include "VideoIOConfiguration.hpp"
#include "ConsoleLogger.hpp"

namespace clt
{

    class VideoProcessor
    {
    public:

        explicit VideoProcessor(VideoIOConfiguration configuration);

        std::optional<std::string> run();

        void stop();

    private:

        std::optional<std::string> initialize_configuration();

        std::optional<std::string> initialize_output_stream(const cv::Size frame_size);

        void write_to_loggers();

        void print_progress();

        void print_filter_timings();

        void log_timing_data();

        static std::string make_progress_bar(const uint32_t length, const double progress);

    private:
        VideoIOConfiguration m_Configuration;
        bool m_DeviceCapture = false;

        std::ofstream m_DataLogStream;
        std::optional<lvk::CSVLogger> m_DataLogger;
        ConsoleLogger m_ConsoleLogger;

        cv::VideoCapture m_InputStream;
        cv::VideoWriter m_OutputStream;
        lvk::CompositeFilter m_Processor;

        bool m_Terminate = false;
        lvk::TickTimer m_FrameTimer;
        lvk::Stopwatch m_ProcessTimer;
    };

}
