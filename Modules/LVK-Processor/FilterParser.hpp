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

#include "OptionParser.hpp"

#include <deque>
#include <LiveVisionKit.hpp>

namespace clt
{

    class FilterParser : private OptionsParser
    {
    public:

        std::shared_ptr<lvk::VideoFilter> try_parse(std::deque<std::string>& args);

        template<typename F, typename C>
        void add_filter(
            const std::initializer_list<std::string>& aliases,
            const std::string& description,
            const std::function<void(OptionsParser&, C&)>& config_connector
        );

        template<typename F, typename C>
        void add_filter(
            const std::string& name,
            const std::string& description,
            const std::function<void(OptionsParser&, C&)>& config_connector
        );

        bool has_filter(const std::string& aliases) const;

        const std::string& config_manual(const std::string& filter) const;

        std::string manual(const std::string& filter) const;

        const std::string& manual() const;

        void set_error_handler(const ErrorHandler& handler);

    private:

        template<typename C>
        void generate_config_manual(
            const std::initializer_list<std::string>& aliases,
            const std::function<void(OptionsParser&, C&)>& config_connector
        );

        template<typename F, typename C>
        void generate_filter_constructor(
            const std::function<void(OptionsParser&, C&)>& config_connector
        );

    private:

        struct ConfigurableFilter
        {
            std::shared_ptr<lvk::VideoFilter> instance;
            std::function<void(std::deque<std::string>&)> configure;
        };

        using FilterConstructor = std::function<ConfigurableFilter()>;
        std::unordered_map<std::string, FilterConstructor> m_FilterConstructors;

        FilterConstructor m_ParsedConstructor;
        ErrorHandler m_ErrorHandler = [](auto&, auto&){};

        std::vector<std::string> m_ConfigManuals;
        std::unordered_map<std::string, size_t> m_ManualLookup;

    };

}

#include "FilterParser.tpp"