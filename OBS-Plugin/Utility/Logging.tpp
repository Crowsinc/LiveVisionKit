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

#include <string>
#include <obs-module.h>

namespace lvk::log
{

//---------------------------------------------------------------------------------------------------------------------

	template<typename... T>
	void log(const int log_level, const std::string& format, T... args)
	{
		blog(log_level, ("[LiveVisionKit] " + format).c_str(), args...);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename... T>
	void log_if(const bool condition, const int log_level, const std::string& format, T... args)
	{
		if (condition)
			log(log_level, format.c_str(), args...);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename... T>
	void log_block(const int log_level, const std::string& format, T... args)
	{
		blog(log_level, "==== [LiveVisionKit] ======================================");
		blog(log_level, format.c_str(), args...);
		blog(log_level, "===========================================================");
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename... T>
	void print(const std::string& format, T... args)
	{
		log(LOG_INFO, format.c_str(), args...);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename... T>
	void print_if(const bool condition, const std::string& format, T... args)
	{
		log_if(condition, LOG_INFO, format.c_str(), args...);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename... T>
	void print_block(const std::string& format, T... args)
	{
		log_block(LOG_INFO, format, args...);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename... T>
	void warn(const std::string& format, T... args)
	{
		log(LOG_WARNING,format, args...);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename... T>
	void warn_if(const bool condition, const std::string& format, T... args)
	{
		log_if(condition, LOG_WARNING, format, args...);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename... T>
	void warn_block(const std::string& format, T... args)
	{
		log_block(LOG_WARNING, format, args...);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename... T>
	void error(const std::string& format, T... args)
	{
		log(LOG_ERROR, format, args...);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename... T>
	void error_if(const bool condition, const std::string& format, T... args)
	{
		log_if(condition, LOG_ERROR, format, args...);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename... T>
	void error_block(const std::string& format, T... args)
	{
		log_block(LOG_ERROR, format, args...);
	}

//---------------------------------------------------------------------------------------------------------------------

}