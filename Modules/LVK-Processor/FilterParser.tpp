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

#include <type_traits>

#include <iostream>

namespace clt
{
//---------------------------------------------------------------------------------------------------------------------

    std::shared_ptr<lvk::VideoFilter> FilterParser::try_parse(std::deque<std::string>& args)
    {
        if(args.empty())
            return nullptr;

        const auto filter_name = std::string(args.front());

        if(has_filter(filter_name))
        {
            args.pop_front();

            auto filter = m_FilterConstructors[filter_name]();
            filter.configure(args);

            return filter.instance;
        }

        return nullptr;
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename F, typename C>
    void FilterParser::add_filter(
        const std::initializer_list<std::string>& alias,
        const std::function<void(OptionsParser&, C&)>& config_connector
    )
    {
        static_assert(std::is_base_of_v<lvk::VideoFilter, F>, "Filter is not a derivation of VideoFilter");
        static_assert(std::is_base_of_v<lvk::Configurable<C>, F>, "Filter is not a derivation of Configurable<C>");
        for(const auto& name : alias)
        {
            m_FilterConstructors[name] = [=](){
                ConfigurableFilter filter;
                filter.instance = std::make_shared<F>();
                filter.configure = [=](std::deque<std::string>& args) {
                    C config;
                    OptionsParser config_parser;
                    config_connector(config_parser, config);

                    while(config_parser.try_parse(args));

                    std::static_pointer_cast<lvk::Configurable<C>>(
                        std::static_pointer_cast<F>(filter.instance)
                    )->configure(config);
                };
                return filter;
            };
        }
    }


//---------------------------------------------------------------------------------------------------------------------

    template<typename F, typename C>
    void FilterParser::add_filter(
        const std::string& name,
        const std::function<void(OptionsParser&, C&)>& config_connector
    )
    {
        add_filter({name}, config_connector);
    }

//---------------------------------------------------------------------------------------------------------------------

    bool FilterParser::has_filter(const std::string& alias) const
    {
        return m_FilterConstructors.contains(alias);
    }

//---------------------------------------------------------------------------------------------------------------------
}
