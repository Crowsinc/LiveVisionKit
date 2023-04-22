//    *************************** LiveVisionKit ****************************
//    Copyright (C) 2022  Sebastian Di Marco (crowsinc.dev@gmail.com)
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.
// 	  **********************************************************************

#pragma once

#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <functional>

#include "Directives.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

    template<typename T>
    inline std::vector<T> parse_sequence(
        const std::string& string,
        const char delimiter,
        const std::function<bool(size_t,T&,bool)>& validate
    )
    {
        static_assert(std::is_fundamental_v<T>);

        std::stringstream parser, stream(string);
        std::string token;

        size_t index = 0;
        std::vector<T> output;
        while(std::getline(stream, token, delimiter))
        {
            parser.clear();
            parser.str(token);

            T value = T{};
            parser >> value;

            if(validate(index, value, parser.fail()))
                output.emplace_back(value);

            index++;
        }

        return output;
    }

//---------------------------------------------------------------------------------------------------------------------

}
