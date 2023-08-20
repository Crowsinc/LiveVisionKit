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

namespace lvk
{
	template<typename T>
	bool test_bits(const T bits, const T test_flag);

	template<typename T, typename ...Options>
	bool any_of(const T value, Options ...option);

	template<typename T, typename ...Options>
	bool all_of(const T value, Options ...option);


    template<typename T>
    T hysteresis(const T state, const T thresh_lower, const T state_lower, const T thresh_upper, const T state_upper);

}

#include "Logic.tpp"
