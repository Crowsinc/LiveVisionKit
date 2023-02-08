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

#include <LiveVisionKit.hpp>
#include <signal.h>

#include "VideoIOConfiguration.hpp"
#include "VideoProcessor.hpp"

#ifdef WIN32
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <unistd.h>
#endif


std::function<void()> signal_handler;

int main(int argc, char* argv[])
{
    clt::VideoIOConfiguration configuration;

    // If no arguments were provided, print the manual
    if(argc == 1)
    {
        configuration.print_manual();
        return 0;
    }

    // Parse the command-line configuration
    if(auto error = configuration.from_command_line(argc, argv); error.has_value())
    {
        std::cerr << error.value() << "\n";
        return 1;
    }

    clt::VideoProcessor processor(configuration);

    // Set up signal to terminate the processor early on ctrl+c
    signal_handler = [&](){
        processor.stop();
    };
    signal(SIGINT, [](int s){signal_handler();});

    // Set up LVK assert handler
    lvk::global::assert_handler = [](auto, auto, const std::string& assertion){
        std::cerr << cv::format("LiveVisionKit failed condition: %s\n", assertion.c_str());
        std::abort();
    };

    // Set process priority
#ifdef WIN32
    SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS);
#else
    nice(-40);
#endif


    // Run the video processor
    if(auto error = processor.run(); error.has_value())
    {
        std::cerr << error.value() << "\n";
        return 1;
    }

    return 0;
}
