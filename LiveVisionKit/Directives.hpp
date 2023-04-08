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
#include <fstream>
#include <functional>

// UTILITY

// taken from https://stackoverflow.com/a/8488201
#ifdef _WIN32
#define LVK_FILE (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
#else 
#define LVK_FILE (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#endif


// ASSERTS

namespace lvk::context
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
        lvk::context::assert_handler(LVK_FILE, __func__, #assertion);                                                  \
	}

#define LVK_ASSERT_IF(condition, assertion)                                                                            \
	if((condition) && !(assertion))                                                                                    \
	{                                                                                                                  \
        lvk::context::assert_handler(LVK_FILE, __func__, #assertion);                                                  \
	}

#define LVK_ASSERT_01(value)                                                                                           \
	if(value < 0 || value > 1)                                                                                         \
	{                                                                                                                  \
        lvk::context::assert_handler(LVK_FILE, __func__, "0 <= #value <= 1");                                          \
	}

#define LVK_ASSERT_01_STRICT(value)                                                                                    \
	if(value <= 0 || value >= 1)                                                                                       \
	{                                                                                                                  \
        lvk::context::assert_handler(LVK_FILE, __func__, "0 < #value < 1");                                            \
	}

#define LVK_ASSERT_RANGE(value, min, max)                                                                              \
	if(value < min || value > max)                                                                                     \
	{                                                                                                                  \
        lvk::context::assert_handler(LVK_FILE, __func__, "#min <= #value <= #max");                                    \
	}

#define LVK_ASSERT_RANGE_STRICT(value, min, max)                                                                       \
	if(value <= min || value >= max)                                                                                   \
	{                                                                                                                  \
        lvk::context::assert_handler(LVK_FILE, __func__, "#min < #value < #max");                                      \
	}


#else

#define LVK_ASSERT(assertion)
#define LVK_ASSERT_IF(condition, assertion)

#define LVK_ASSERT_01(value)
#define LVK_ASSERT_01_STRICT(value)
#define LVK_ASSERT_RANGE(value, min, max)
#define LVK_ASSERT_RANGE_STRICT(value, min, max)

#endif


// LOGGING

// NOTE: Resulting CSVLogger will be named '_var'
#define INIT_CSV(var, path)                                                                                            \
    static std::ofstream log_file_##var;                                                                               \
    static bool log_init_##var = false;                                                                                \
    if(!log_init_##var)                                                                                                \
    {                                                                                                                  \
        log_file_##var.open(path);                                                                                     \
        log_init_##var = true;                                                                                         \
                                                                                                                       \
        LVK_ASSERT(log_file_##var.good())                                                                              \
    }                                                                                                                  \
    static lvk::CSVLogger _##var(log_file_##var)
