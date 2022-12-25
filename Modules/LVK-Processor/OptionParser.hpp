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

#include <deque>
#include <string>
#include <optional>
#include <functional>
#include <unordered_map>
#include <LiveVisionKit.hpp>

namespace clt
{

    using ArgQueue = std::deque<std::string>;

    class OptionsParser
    {
    public:

        bool try_parse(ArgQueue& args) const;

        template<typename T>
        void add_variable(
            const std::string& name,
            const std::string& description,
            T* location
        );

        template<typename T>
        void add_variable(
            const std::initializer_list<std::string>& aliases,
            const std::string& description,
            T* location
        );

        template<typename T>
        void add_variable(
            const std::string& name,
            const std::string& description,
            const std::function<void(T)>& callback
        );

        template<typename T>
        void add_variable(
            const std::initializer_list<std::string>& aliases,
            const std::string& description,
            const std::function<void(T)>& callback
        );

        void add_switch(
            const std::string& name,
            const std::string& description,
            bool* location
        );

        void add_switch(
            const std::initializer_list<std::string>& aliases,
            const std::string& description,
            bool* location
        );

        void add_switch(
            const std::string& name,
            const std::string& description,
            const std::function<void()>& callback = {}
        );

        void add_switch(
            const std::initializer_list<std::string>& aliases,
            const std::string& description,
            const std::function<void()>& callback = {}
        );

        void add_parser(
            const std::string& name,
            const std::string& description,
            const std::function<bool(ArgQueue&)>& parser
        );

        void add_parser(
            const std::initializer_list<std::string>& aliases,
            const std::string& description,
            const std::function<bool(ArgQueue&)>& parser
        );

        const std::string& manual() const;

        std::string manual(const std::string& option) const;

        bool has_variable(const std::string& name) const;

        bool has_switch(const std::string& name) const;

        bool has_parser(const std::string& name) const;

        bool is_empty() const;

        using ErrorHandler = std::function<void(
            const std::string& alias,
            const std::string& value
        )>;

        void set_error_handler(const ErrorHandler& handler);

    private:

        template<typename T>
        std::optional<T> parse_as(const std::string& argument);

        void generate_manual_entry(
            const std::initializer_list<std::string>& aliases,
            const std::string& description,
            const bool has_arg
        );

        void compile_manual();

    private:

        ErrorHandler m_ErrorHandler = [](auto,auto){};

        using ParserOptionHandler = std::function<bool(ArgQueue& queue)>;
        std::unordered_map<std::string, ParserOptionHandler> m_ParserOptions;

        using VariableOptionHandler = std::function<bool(const std::string&)>;
        std::unordered_map<std::string, VariableOptionHandler> m_VariableOptions;

        using SwitchOptionHandler = std::function<void()>;
        std::unordered_map<std::string, SwitchOptionHandler> m_SwitchOptions;

        std::string m_Manual;
        size_t m_LongestNameEntryLength = 0;
        std::unordered_map<std::string, size_t> m_ManualLookup;
        std::vector<std::tuple<std::string /* Name Entry */, std::string /* Description */>> m_ManualEntries;
    };

}

#include "OptionParser.tpp"