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

#include <sstream>

namespace clt
{

//---------------------------------------------------------------------------------------------------------------------

    bool OptionsParser::try_parse(std::deque<std::string>& args) const
    {
        if(args.empty())
            return false;

        const auto option = args.front();

        // NOTE: For duplicate alias between variabel and switch type options, we
        // prioritise parsing as a variable first, then as a switch if it fails.

        if(has_variable(option) && args.size() >= 2)
        {
            const auto argument = args[1];

            // Only 'consume' the arguments if successful
            if(m_VariableOptions.at(option)(argument))
            {
                args.pop_front();
                args.pop_front();
                return true;
            }
        }

        if(has_switch(option))
        {
            m_SwitchOptions.at(option)();
            args.pop_front();
            return true;
        }

        return false;
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    std::optional<T> OptionsParser::parse_as(const std::string& argument)
    {
        std::stringstream parser(argument);

        T value;
        parser >> value;

        if(parser.fail())
            return std::nullopt;
        else
            return value;
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    void OptionsParser::add_variable(
        const std::initializer_list<std::string>& alias,
        const std::function<void(T)>& callback
    )
    {
        for(const auto& name : alias)
        {
            m_VariableOptions[name] = [=](const std::string& argument)
            {
                std::optional<T> parsed_argument = parse_as<T>(argument);

                if(parsed_argument.has_value())
                {
                    callback(parsed_argument.value());
                    return true;
                }
                else return false;
            };
        }
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    void OptionsParser::add_variable(
        const std::string& name,
        const std::function<void(T)>& callback
    )
    {
        add_variable({name}, callback);
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    void OptionsParser::add_variable(
        const std::initializer_list<std::string>& alias,
        T* location
    )
    {
        for(const auto& name : alias)
        {
            m_VariableOptions[name] = [=](const std::string& argument)
            {
                std::optional<T> parsed_argument = parse_as<T>(argument);

                if(parsed_argument.has_value())
                {
                    *location = parsed_argument.value();
                    return true;
                }
                else return false;
            };
        }
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    void OptionsParser::add_variable(
        const std::string& name,
        T* location
    )
    {
        add_variable({name}, location);
    }

//---------------------------------------------------------------------------------------------------------------------

    void OptionsParser::add_switch(
        const std::initializer_list<std::string>& alias,
        bool* location
    )
    {
        for(const auto& name : alias)
        {
            m_SwitchOptions[name] = [=]()
            {
                *location = true;
            };
        }
    }

//---------------------------------------------------------------------------------------------------------------------

    void OptionsParser::add_switch(
        const std::string& name,
        bool* location
    )
    {
        add_switch({name}, location);
    }

//---------------------------------------------------------------------------------------------------------------------

    void OptionsParser::add_switch(
        const std::initializer_list<std::string>& alias,
        const std::function<void()>& callback
    )
    {
        for(const auto& name : alias)
        {
            m_SwitchOptions[name] = callback;
        }
    }

//---------------------------------------------------------------------------------------------------------------------

    void OptionsParser::add_switch(
        const std::string& name,
        const std::function<void()>& callback
    )
    {
        add_switch({name}, callback);
    }

//---------------------------------------------------------------------------------------------------------------------

    bool OptionsParser::has_variable(const std::string& alias) const
    {
        return m_VariableOptions.contains(alias);
    }

//---------------------------------------------------------------------------------------------------------------------

    bool OptionsParser::has_switch(const std::string& alias) const
    {
        return m_SwitchOptions.contains(alias);
    }

//---------------------------------------------------------------------------------------------------------------------

    bool OptionsParser::is_empty() const
    {
        return m_SwitchOptions.empty() && m_VariableOptions.empty();
    }

//---------------------------------------------------------------------------------------------------------------------

}

