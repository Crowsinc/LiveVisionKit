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

#include "VideoIOConfiguration.hpp"

#include <fstream>
#include <opencv2/opencv.hpp>

namespace clt
{

//---------------------------------------------------------------------------------------------------------------------

    VideoIOConfiguration::VideoIOConfiguration()
    {
        register_filters();
        register_options();

        // Set error handlers
        m_FilterParser.set_error_handler([this](auto& config, auto& argument)
        {
            m_ParserError = cv::format(
                 "Failed to parse argument \'%s\' for filter config \'%s\'",
                 argument.c_str(),
                 config.c_str()
           );
        });

        m_OptionParser.set_error_handler([this](auto& option, auto& argument)
        {
            m_ParserError = cv::format(
             "Failed to parse argument \'%s\' for option \'%s\'",
             argument.c_str(),
             option.c_str()
            );
        });
    }

//---------------------------------------------------------------------------------------------------------------------

    std::optional<std::string> VideoIOConfiguration::from_command_line(const int argc, char* argv[])
    {
        ArgQueue arguments;
        for(int i = 1; i < argc; i++)
            arguments.emplace_back(argv[i]);

        // Command-line format includes a mandatory input and optional output target
        // declaration, sandwiched between two sets of optional arguments.

        while(m_OptionParser.try_parse(arguments));
        if(m_ParserError.has_value())
            return m_ParserError;

        if(auto error = parse_io_targets(arguments); error.has_value())
            return error;

        while(m_OptionParser.try_parse(arguments));
        if(m_ParserError.has_value())
            return m_ParserError;

        // There will only be arguments left over if they didn't match any known options.
        if(!arguments.empty())
        {
            return cv::format(
                "Unknown argument \'%s\', use -h to see available options",
                arguments.front().c_str()
            );
        }

        return std::nullopt;
    }

//---------------------------------------------------------------------------------------------------------------------

    std::optional<std::string> VideoIOConfiguration::parse_io_targets(ArgQueue& arguments)
    {
        if(arguments.empty())
            return "No input was specified";

        // The first argument is the input.
        auto input = arguments.front();
        arguments.pop_front();

        // Parse the input target
        std::optional<std::string> input_format;
        if(std::filesystem::path path = input; path.has_filename() && path.has_extension())
        {
            // Input is file path
            input_source = path;
        }
        else if(std::all_of(input.begin(), input.end(), [](int c){return std::isalnum(c);}))
        {
            // Input is device specifier
            input_source = static_cast<uint32_t>(std::stoi(input));
        }
        else
        {
            // No input specified
            return cv::format(
                "Unknown input, got \'%s\', expected a file path or integer device specifier",
                input.c_str()
            );
        }

        // Second argument is the optional output.
        if(!arguments.empty())
        {
            // Attempt to parse an output, this is optional so it can safely fail.
            // The output will always be a file path with the same format as the input video
            auto output = std::string(arguments.front());
            if(std::filesystem::path path = output; path.has_filename() && path.has_extension())
            {
                // If the input was a video file, restrict the output to match the file format.
                // This is not an encoding tool, so we can make things easier on ourselves here.
                if(input_format.has_value() && *input_format != path.extension())
                {
                    return cv::format(
                        "Mismatched input and output video formats, output was \'%s\', expected \'%s\'",
                        path.extension().string().c_str(),
                        input_format->c_str()
                    );
                }

                output_target = path;
                arguments.pop_front();
            }
        }

        return std::nullopt;
    }

//---------------------------------------------------------------------------------------------------------------------

    std::optional<std::string> VideoIOConfiguration::parse_profile(ArgQueue& arguments)
    {
        if(arguments.empty())
            return "No profile specified, expected path after -p";

        if(std::filesystem::path path = arguments.front(); std::filesystem::exists(path) && path.has_extension())
        {
            std::ifstream file(path);
            if(!file.good())
                return cv::format("Failed to open profile at \'%s\'", path.string().c_str());
            arguments.pop_front();

            // Read entire profile and split it into individual arguments
            std::stringstream profile;
            profile << file.rdbuf();

            std::vector<std::string> profile_args;
            for(std::string line; std::getline(profile, line, '\n');)
            {
                std::stringstream line_buffer(line);
                for(std::string arg; std::getline(line_buffer, arg, ' ');)
                {
                    if(!arg.empty())
                        profile_args.push_back(arg);
                }
            }

            // Add new args to the front of the queue in reverse so its in the correct order
            // TODO: check if ranges::view::reverse is available on GCC ubuntu 20.04
            for(auto arg_iter = profile_args.rbegin(); arg_iter != profile_args.rend(); ++arg_iter)
                arguments.insert(arguments.begin(), *arg_iter);
        }
        else return cv::format("Profile \'%s\' does not exist", path.string().c_str());

        return std::nullopt;
    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoIOConfiguration::print_filter_manual(const std::string& filter) const
    {
        std::cout << "Filter: \n"
                  << "\t" << m_FilterParser.manual(filter)
                  << "\n\n";

        std::cout << "Configuration Options: \n"
                  << m_FilterParser.config_manual(filter)
                  << "\n";
    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoIOConfiguration::print_manual() const
    {
        std::cout << "\nFormat: lvk [Options...] Input [Output] [Options...]"
                  << "\n\n";

        std::cout << "Where...\n"
                  << "\t * Input may either be an input video file, a set of images, or an index specifying a capture "
                     "device to read from.\n"
                  << "\t * Output is an optional video file path to which filtered video data is written. If paired "
                     "with a video file input, they must be of matching extensions. \n"
                  << "\t * If no output is specified, or a device capture input is used, a display window will be used"
                     " to show output frames. This window can be closed using <escape>, ending all processing."
                  << "\n\n";

        std::cout << "Options: \n"
                  << m_OptionParser.manual()
                  << "\n\n";

        std::cout << "Filters: \n"
                  << m_FilterParser.manual()
                  << "\n\n";
    }

//---------------------------------------------------------------------------------------------------------------------

    void VideoIOConfiguration::register_options()
    {
        // Help / Meta Options

        m_OptionParser.add_switch(
            "-h",
            "Displays the complete configuration manual.",
            [&]() {print_manual();}
        );

        m_OptionParser.add_variable<std::string>(
            "-H",
            "Displays all configuration options for the specified filter.",
            [&](const std::string& filter)
            {
                if(!m_FilterParser.has_filter(filter))
                {
                    m_ParserError = cv::format(
                        "Invalid use of -H, got \'%s\', expected filter name",
                        filter.c_str()
                    );
                    return;
                }

                std::cout << "Filter: \n"
                          << "\t" << m_FilterParser.manual(filter) << "\n\n"
                          << "Configuration Options: \n"
                          << m_FilterParser.config_manual(filter) << "\n";
            }
        );

        m_OptionParser.add_parser(
            "-p",
            "Loads a set of arguments from the specified file, allowing for saved filter sets and profiles",
            [this](ArgQueue& arguments)
            {
                // Pop '-p' from the arguments queue
                arguments.pop_front();

                m_ParserError = parse_profile(arguments);

                return !m_ParserError.has_value();
            }
        );

        // Filter Options

        m_OptionParser.add_parser(
            "-f",
            "Adds a filter used for processing. Filters are processed left to right and can be modified by "
            "supplying configuration options after the filter specification. See the filter listing below and"
            " -H <filter> for more information.",
            [this](ArgQueue& arguments)
            {
                // Pop '-f' from the arguments queue
                arguments.pop_front();

                const auto filter_name = arguments.front();
                if(auto filter = m_FilterParser.try_parse(arguments); filter == nullptr)
                {
                    m_ParserError = cv::format(
                        "Unknown filter \'%s\', use -H to see available options", filter_name.c_str()
                    );
                    return false;
                }
                else
                {
                    filter_chain.push_back(filter);
                    return true;
                }
            }
        );

        m_OptionParser.add_switch(
            "-d",
            "Runs all filters in debug mode, allowing for more "
            "accurate timing data and special filter debug rendering.",
            &debug_mode
        );

        // Output Options
        m_OptionParser.add_variable<int>(
            "-r",
            "Used to specify the desired integer framerate of the output video.",
            [this](const int framerate) {
                if(framerate <= 0)
                {
                    m_ParserError = cv::format(
                        "Output framerate cannot be zero or negative, got \'%d\' FPS",
                        framerate
                    );
                    return;
                }
                output_framerate = framerate;
            }
        );

        m_OptionParser.add_variable<std::string>(
            "-c",
            "Used to suggest the fourcc encoder designation to be used for the output video. "
            "Encoding support is not guaranteed.",
            [this](const std::string& fourcc)
            {
                if(fourcc.length() != 4)
                {
                    m_ParserError = cv::format("Unknown codec, expected fourcc code, got \'%s\'", fourcc.c_str());
                    return;
                }
                output_codec = cv::VideoWriter::fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]);
            }
        );

        m_OptionParser.add_switch(
            "-C",
            "Lists the fourcc codes of all available encoders.",
            [](){
                // Print out encoders using the (-1) fourcc function of the ffmpeg backend
                std::cout << "Encoders:\n";
                cv::VideoWriter("test.mp4", cv::CAP_FFMPEG, -1, 1, cv::Size(600,400));
                std::cout << "\n";
            }
        );

        m_OptionParser.add_switch(
            "-s",
            "Renders the processor output onto an interactable window that can be closed by pressing escape.",
            &render_output
        );

        m_OptionParser.add_variable<int>(
            "-S",
            "Equivalent to -s, but locks the maximum processing framerate to the given amount.",
            [this](const int framerate){
                if(framerate <= 0)
                {
                    m_ParserError = cv::format(
                        "Display framerate cannot be zero or negative, got \'%d\' FPS",
                        framerate
                    );
                    return;
                }
                render_output = true;
                render_period = lvk::Time::Timestep(static_cast<double>(framerate));
            }
        );

        // Logging Options

        m_OptionParser.add_variable<double>(
            "-u",
            "Used to specify the numeric amount of seconds to wait between each logging operation.",
            [this](double seconds) {
                if(seconds <= 0)
                {
                    m_ParserError = cv::format(
                        "Update period cannot be zero or negative, got \'%.2f\' seconds",
                        seconds
                    );
                    return;
                }

                update_period = lvk::Time::Seconds(seconds);
            }
        );

        m_OptionParser.add_switch(
            "-v",
            "Enables the display of extra runtime information such as filter runtimes and configurations.",
            &print_timings
        );

        m_OptionParser.add_variable<std::string>(
            "-L",
            "Turns on filter timing-data logging to the specified CSV filepath.",
            [this](const std::string& path_arg)
            {
                const std::filesystem::path path = path_arg;
                if(path.extension() != ".csv")
                {
                    m_ParserError = cv::format(
                        "Invalid data logging target, got file type %s, expected \'.csv\'",
                        path.extension().string().c_str()
                    );
                }
                log_target = path;
            }
        );
    }

//---------------------------------------------------------------------------------------------------------------------

    // TODO: add testing of config values
    void VideoIOConfiguration::register_filters()
    {
        m_FilterParser.add_filter<lvk::StabilizationFilter, lvk::StabilizationFilterSettings>(
            {"vs", "stab"},
            "A video stabilization filter used to smoothen percieved camera motions.",
            [](clt::OptionsParser& config_parser, lvk::StabilizationFilterSettings& config){
                config_parser.add_variable(
                    {".crop_prop", ".cp"},
                    "Used to percentage crop and movement area allowed for stabilization",
                    &config.scene_margins
                );
                config_parser.add_switch(
                    {".crop_out", ".co"},
                    "Specifies that the output should be automatically cropped",
                    &config.crop_frame_to_margins
                );
                config_parser.add_variable(
                    {".smoothing", ".s"},
                    "The amount of camera smoothing to apply to the video.",
                    &config.path_prediction_frames
                );
            }
        );

        m_FilterParser.add_filter<lvk::DeblockingFilter, lvk::DeblockingFilterSettings>(
            {"adb", "deblocker"},
            "An adaptive deblocking filter used to lessen the effect of blocking encoding artifacts.",
            [](clt::OptionsParser& config_parser, lvk::DeblockingFilterSettings& config){
                config_parser.add_variable(
                    {".levels", ".l"},
                    "Used to specify the number of deblocking passes to perform.",
                    &config.detection_levels
                );
            }
        );
    }

//---------------------------------------------------------------------------------------------------------------------

}