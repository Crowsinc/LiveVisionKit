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
#include <cstring>
#include <functional>

// taken from https://stackoverflow.com/a/8488201
#ifdef _WIN32
#define LVK_FILE (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
#else 
#define LVK_FILE (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#endif

namespace lvk::global
{
    extern std::function<void(
        std::string file,
        std::string function,
        std::string assertion
    )> assert_handler;
}

#ifndef LVK_DISABLE_CHECKS

#define LVK_ASSERT(assertion)																						   \
	if(!(assertion))																								   \
	{                                                                                                                  \
        lvk::global::assert_handler(LVK_FILE, __func__, #assertion);                                                   \
	}
#else

#define LVK_ASSERT(assertion)

#endif


