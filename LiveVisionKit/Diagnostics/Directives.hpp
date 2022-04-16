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

#include <obs.h>
#include <string>

// taken from https://stackoverflow.com/a/8488201
#ifdef _WIN32
#define LVK_FILE (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
#else 
#define LVK_FILE (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#endif

#ifndef DISABLE_CHECKS

#define LVK_ASSERT(assertion) if(!(assertion)){bcrash("LVK@%s@%s(..) FAILED " #assertion, LVK_FILE, __func__);}

#define LVK_CRASH(msg) bcrash("LVK@%s@%s(..) CRASHED " #msg, LVK_FILE, __func__);

#define LVK_WARN(msg) blog(LOG_WARNING, "LVK@%s@%s(..) WARNING " #msg, LVK_FILE, __func__)

#define LVK_WARN_IF(pred, msg) if(pred){blog(LOG_WARNING, "LVK@%s@%s(..) WARNING " #msg, LVK_FILE, __func__);}

#define LVK_ERROR(msg) blog(LOG_ERROR, "LVK@%s@%s(..) ERROR " #msg, LVK_FILE, __func__);

#define LVK_ERROR_IF(pred, msg) if(pred){blog(LOG_ERROR, "LVK@%s@%s(..) ERROR " #msg, LVK_FILE, __func__);}

#else

#define LVK_ASSERT(assertion)

#define LVK_CRASH(msg)

#define LVK_WARN(msg)

#define LVK_WARN_IF(pred, msg)

#define LVK_ERROR(msg)

#define LVK_ERROR_IF(pred, msg)

#endif


