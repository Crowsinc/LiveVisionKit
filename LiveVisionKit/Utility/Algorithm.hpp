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

namespace lvk
{

	// Erase quickly without care for preserving the element ordering
	template<typename T>
	void fast_erase(std::vector<T>& data, const size_t index);

	// Filter quickly without care for preserving the element ordering
	template<typename T, typename P>
	void fast_filter(std::vector<T>& data, const std::vector<P>& keep, bool invert = false);

	// Filter quickly without care for preserving the element ordering
	template<typename T, typename P>
	void fast_filter(std::vector<T>& data_1, std::vector<T>& data_2, const std::vector<P>& keep, bool invert = false);


	template<typename T, typename P>
	void filter(std::vector<T>& data, const std::vector<P>& keep, bool invert = false);

}

#include "Algorithm.tpp"
