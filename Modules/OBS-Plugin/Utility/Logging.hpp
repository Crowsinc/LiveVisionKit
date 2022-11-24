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

namespace lvk::log
{

	template<typename... T>
	void log(const int log_level, const std::string& format, T... args);

	template<typename... T>
	void log_if(const bool condition, const int log_level, const std::string& format, T... args);

	template<typename... T>
	void log_block(const int log_level, const std::string& format, T... args);


	template<typename... T>
	void print(const std::string& format, T... args);

	template<typename... T>
	void print_if(const bool condition, const std::string& format, T... args);

	template<typename... T>
	void print_block(const std::string& format, T... args);


	template<typename... T>
	void warn(const std::string& format, T... args);

	template<typename... T>
	void warn_if(const bool condition, const std::string& format, T... args);

	template<typename... T>
	void warn_block(const std::string& format, T... args);


	template<typename... T>
	void error(const std::string& format, T... args);

	template<typename... T>
	void error_if(const bool condition, const std::string& format, T... args);

	template<typename... T>
	void error_block(const std::string& format, T... args);

}

#include "Logging.tpp"

