
#include <opencv2/videoio.hpp>
#include <LiveVisionKit.hpp>

#include <unordered_map>
#include <string_view>
#include <filesystem>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <deque>

#include "OptionParser.hpp"
#include "FilterParser.hpp"

/* TODO:
 * 1. Fix error on a bad device capture stream (e.g. using 2)
 * 2. Finalize console output
 * 3. Add more options, e.g. logging, device capture resolution, etc.
 * ?. Write help manual
 * ?. Add proper error message handling within parsers
 * ?. Add all filters and options
 */

using ArgQueue = std::deque<std::string>;
using FilterChain = std::vector<std::shared_ptr<lvk::VideoFilter>>;

// Helper Functions

void print_manual();

template<typename... T>
void abort_with_error(const char* fmt, T... args);

// Processing Functions

struct Settings
{
    bool debug_mode = false;
    bool display_output = false;

    // TODO: std::optional<std::filesystem::path> log_output;

    bool is_device_capture = false;
    cv::VideoCapture input_stream;
    lvk::CompositeFilter video_processor;
    std::optional<std::filesystem::path> output_path;
    int output_codec = cv::VideoWriter::fourcc('H','2','6','4');
    double output_fps = 0;
};

void register_filters(clt::FilterParser& parser);

void register_options(clt::OptionsParser& parser, Settings& settings);

void parse_io(ArgQueue& args, Settings& settings);

void parse_profile(ArgQueue& args);

void finalize_filters(FilterChain& filter_chain, Settings& settings);

void run_video_processor(Settings& settings);

int main(int argc, char* argv[])
{
    Settings settings;

    // INITIALIZATION
    clt::OptionsParser option_parser;
    register_options(option_parser, settings);

    clt::FilterParser filter_parser;
    register_filters(filter_parser);

    ArgQueue args;
    for(int i = 1; i < argc; i++)
        args.emplace_back(argv[i]);

    parse_io(args, settings);

    FilterChain filter_chain;

    // Parse filters and options
    while(!args.empty())
    {
        if(args.front().starts_with("-p"))
        {
            args.pop_front();
            parse_profile(args);
        }
        else if(args.front().starts_with("-f"))
        {
            // Parse filter & filter config options
            args.pop_front();
            if(auto filter = filter_parser.try_parse(args); filter != nullptr)
                filter_chain.push_back(filter);
            else abort_with_error("Unknown filter \'%s\', use -h to see available options", args.front().c_str());
        }
        else if(!option_parser.try_parse(args))
            abort_with_error("Unknown argument \'%s\', use -h to see available options", args.front().c_str());
    }

    finalize_filters(filter_chain, settings);

    // Run the processor
    run_video_processor(settings);

    return 0;
}

//---------------------------------------------------------------------------------------------------------------------

void print_manual()
{
    std::cout << "Manual..." << std::endl;
}

//---------------------------------------------------------------------------------------------------------------------

template<typename... T>
void abort_with_error(const char* fmt, T... args)
{
    std::cerr << "ERROR: " << cv::format(fmt, args...) << "\n\n" << std::endl;
    print_manual();
    abort();
}

//---------------------------------------------------------------------------------------------------------------------

void parse_io(ArgQueue& args, Settings& settings)
{
    if(args.empty()) abort_with_error("No input specified");

    // The first argument is the input.
    // This is either a device number, or a file path.
    auto input = std::string(args.front());
    args.pop_front();

    std::optional<std::string> input_format;
    if(std::filesystem::path path = input; path.has_filename() && path.has_extension())
    {
        // If input is a file path, then it is either a video file or an image set
        settings.input_stream = cv::VideoCapture(path.string());
        input_format = path.extension().string();
    }
    else if(std::all_of(input.begin(), input.end(), [](int c){return std::isalnum(c);}))
    {
        // If the input is an alphanumeric number, then it is a device specifier
        settings.input_stream = cv::VideoCapture(std::stoi(input));
        settings.is_device_capture = true;
    }
    else abort_with_error(
        "Unknown input, got \'%s\', expected a file path or integer device specifier",
        input.c_str()
    );

    // Test that the input stream opened successfully
    if(!settings.input_stream.isOpened())
        abort_with_error("Failed to open the input \'%s\'", input.c_str());

    // Second argument is the optional output.
    if(!args.empty())
    {
        // Attempt to parse an output, this is optional so it can safely fail.
        // The output will always be a file path with the same format as the input video
        auto opt_output = std::string(args.front());
        if(std::filesystem::path path = opt_output; path.has_filename() && path.has_extension())
        {
            if(input_format.has_value() && input_format.value() != path.extension())
            {
                abort_with_error(
                    "Mismatched input and output video formats, output was \'%s\', expected \'%s\'",
                    path.extension().string().c_str(),
                    input_format.value().c_str()
                );
            }

            settings.output_path = path;
            args.pop_front();

            // Auto-derive output stream settings from the input
            settings.output_fps = std::max(settings.input_stream.get(cv::CAP_PROP_FPS), 0.0);
            settings.output_codec = static_cast<int>(settings.input_stream.get(cv::CAP_PROP_FOURCC));
        }
    }

    // If its a device capture or there is no output, then we must run with the output display
    if(!settings.output_path.has_value() || settings.is_device_capture)
        settings.display_output = true;
}

//---------------------------------------------------------------------------------------------------------------------

void parse_profile(ArgQueue& args)
{
    if(args.empty()) abort_with_error("No profile specified, expected path after -p");

    if(std::filesystem::path path = args.front(); std::filesystem::exists(path) && path.has_extension())
    {
        std::ifstream file(path);
        if(!file.good())
            abort_with_error("Failed to open profile at \'%s\'", path.string().c_str());

        args.pop_front();

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
            args.push_front(*arg_iter);
    }
    else abort_with_error("Profile \'%s\' does not exist", path.string().c_str());
}

//---------------------------------------------------------------------------------------------------------------------

void finalize_filters(FilterChain& filter_chain, Settings& settings)
{
    // All LVK filters operate on YUV frames as a standard, however
    // most OpenCV video captures and writers provide BGR frames. So
    // we must add conversion filters as necessary.

    settings.video_processor.reconfigure([&](auto& composite_settings){
        composite_settings.filter_chain.emplace_back(
            new lvk::ConversionFilter({.conversion_code=cv::COLOR_BGR2YUV})
        );

        for(auto& filter : filter_chain)
            composite_settings.filter_chain.push_back(filter);

        composite_settings.filter_chain.emplace_back(
            new lvk::ConversionFilter({.conversion_code=cv::COLOR_YUV2BGR})
        );
    });
}

//---------------------------------------------------------------------------------------------------------------------

void run_video_processor(Settings& settings)
{
    cv::VideoWriter output_stream;
    const bool has_output = settings.output_path.has_value();

    lvk::TickTimer frame_timer(std::min<uint32_t>(30, settings.input_stream.get(cv::CAP_PROP_FPS)));

    settings.video_processor.process(
        settings.input_stream,
        [&](lvk::VideoFilter& filter, lvk::Frame& frame)
        {
            // Write to output if one was specified
            if(has_output)
            {
                // Initialize the output on the first frame to get sizing information
                if(!output_stream.isOpened())
                {
                    output_stream = cv::VideoWriter(
                        settings.output_path->string(),
                        settings.output_codec,
                        settings.output_fps,
                        frame.size(),
                        true
                    );

                    if(!output_stream.isOpened())
                    {
                        abort_with_error(
                            "Failed to create output stream",
                            settings.output_path->string().c_str()
                        );
                    }
                }
                output_stream.write(frame.data);
            }

            // Update graphical display
            if(settings.display_output)
            {
                cv::imshow("LVK Output", frame.data);
                int key = cv::pollKey();

                // Close display if escape is pressed
                if(key == 27)
                {
                    settings.display_output = false;
                    cv::destroyAllWindows();

                    // If the input is a device capture or there is no output path, then
                    // we consider the display to the output. So closing the window should
                    // also terminate the processing. This is so that we can decide when to
                    // end indefinite device capture streams, and to avoid accidentally
                    // leaving the processor running in the background indefinitely.
                    return !has_output || settings.is_device_capture;
                }
            }

            // Update console display
            frame_timer.tick();
            std::cout << "Processing Frame " << frame_timer.tick_count() << " at a rate of "
                      << static_cast<uint32_t>(frame_timer.average().frequency()) << "FPS";
            if(!settings.is_device_capture && has_output)
            {
                std::cout << " | Progress...";

                // Add progress bar for video to video processing
                double curr_position = settings.input_stream.get(cv::CAP_PROP_POS_FRAMES);
                double total_frames = settings.input_stream.get(cv::CAP_PROP_FRAME_COUNT);
                double progress = curr_position / total_frames;

                const size_t bar_length = 30;
                std::cout << "   [";
                for(size_t i = 0; i < bar_length; i++)
                    std::cout << ((i < progress * bar_length) ? "=" : " ");
                std::cout << "]";

            };
            std::cout << "\r";

            return false;
        },
        settings.debug_mode
    );

    std::cout << "Completed!" << '\r';
}

//---------------------------------------------------------------------------------------------------------------------

void register_filters(clt::FilterParser& parser)
{
    parser.add_filter<lvk::StabilizationFilter, lvk::StabilizationSettings>(
        {"vs", "stab"},
        [](clt::OptionsParser& config_parser, lvk::StabilizationSettings& config){
            config_parser.add_variable({".crop_prop", ".cp"}, &config.crop_proportion);
            config_parser.add_switch({".crop_out", ".co"}, &config.crop_output);
        }
    );

    parser.add_filter<lvk::DeblockingFilter, lvk::DeblockingSettings>(
        {"adb", "deblocker"},
        [](clt::OptionsParser& config_parser, lvk::DeblockingSettings& config){
            config_parser.add_variable({".strength", ".str"}, &config.detection_levels);
        }
    );
}

//---------------------------------------------------------------------------------------------------------------------

void register_options(clt::OptionsParser& parser, Settings& settings)
{
    parser.add_switch("-d", &settings.debug_mode);
    parser.add_switch("-s", &settings.display_output);
    parser.add_variable("-r", &settings.output_fps);

    parser.add_variable<std::string>("-c", [&](std::string fourcc){
        if(fourcc.length() != 4)
            abort_with_error("Unknown codec, expected fourcc code, got \'%s\'", fourcc.c_str());

        settings.output_codec = cv::VideoWriter::fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]);
    });
}

//---------------------------------------------------------------------------------------------------------------------
