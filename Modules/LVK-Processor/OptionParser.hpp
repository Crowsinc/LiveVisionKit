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

    class OptionsParser
    {
    public:

        bool try_parse(std::deque<std::string>& args) const;

        template<typename T>
        void add_variable(
            const std::string& name,
            T* location
        );

        template<typename T>
        void add_variable(
            const std::initializer_list<std::string>& alias,
            T* location
        );

        template<typename T>
        void add_variable(
            const std::string& name,
            const std::function<void(T)>& callback
        );

        template<typename T>
        void add_variable(
            const std::initializer_list<std::string>& alias,
            const std::function<void(T)>& callback
        );

        void add_switch(
            const std::string& name,
            bool* location
        );

        void add_switch(
            const std::initializer_list<std::string>& alias,
            bool* location
        );

        void add_switch(
            const std::string& name,
            const std::function<void()>& callback = {}
        );

        void add_switch(
            const std::initializer_list<std::string>& alias,
            const std::function<void()>& callback = {}
        );

        bool has_variable(const std::string& alias) const;

        bool has_switch(const std::string& alias) const;

        bool is_empty() const;

    private:

        template<typename T>
        std::optional<T> parse_as(const std::string& argument);

    private:
        using VariableOptionHandler= std::function<bool(const std::string&)>;
        std::unordered_map<std::string, VariableOptionHandler> m_VariableOptions;

        using SwitchOptionHandler = std::function<void()>;
        std::unordered_map<std::string, SwitchOptionHandler> m_SwitchOptions;
    };

}

#include "OptionParser.tpp"