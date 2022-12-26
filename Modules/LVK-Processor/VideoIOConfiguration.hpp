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
#include <filesystem>
#include <optional>
#include <variant>
#include <deque>

#include "OptionParser.hpp"
#include "FilterParser.hpp"

namespace clt
{

    struct VideoIOConfiguration
    {
        // Input / Process Settings
        std::variant<std::monostate, std::filesystem::path, uint32_t> input_source;
        std::vector<std::shared_ptr<lvk::VideoFilter>> filter_chain;
        bool debug_mode = false;

        // Output Settings
        std::optional<std::filesystem::path> output_target;
        std::optional<double> output_framerate;
        std::optional<int> output_codec;

        bool render_output = false;
        std::optional<lvk::Time> render_period;

        // Runtime Settings
        bool print_progress = true;
        bool print_timings = false;
        std::optional<std::filesystem::path> log_target;

        lvk::Time update_period = lvk::Time::Seconds(0.5);

    public:

        VideoIOConfiguration();

        std::optional<std::string> from_command_line(const int argc, char* argv[]);

        void print_filter_manual(const std::string& filter) const;

        void print_manual() const;

    private:

        void register_options();

        void register_filters();

        std::optional<std::string> parse_io_targets(ArgQueue& arguments);

        std::optional<std::string> parse_profile(ArgQueue& arguments);

    private:
        OptionsParser m_OptionParser;
        FilterParser m_FilterParser;

        std::optional<std::string> m_ParserError;
    };

}
