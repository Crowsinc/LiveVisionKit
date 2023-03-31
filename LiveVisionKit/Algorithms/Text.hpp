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

#include <string>
#include <algorithm>
#include <functional>

namespace lvk
{

    template<typename T>
    std::vector<T> parse_sequence(
        const std::string& string,
        const char delimiter = ',',
        const std::function<bool(size_t,T&,bool)>& validate = [](auto index, auto value, auto failed){
            return !failed;
        }
    );

}

#include "Text.tpp"
