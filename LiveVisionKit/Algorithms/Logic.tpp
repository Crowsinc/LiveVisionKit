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

#include <type_traits>

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------
	
	template<typename T>
	inline bool test_bits(const T bits, const T test_flag)
	{
		static_assert(std::is_integral_v<T>);

		return (bits & test_flag) == test_flag;
	}
	
//---------------------------------------------------------------------------------------------------------------------

	template<typename T, typename ...Options>
    inline bool any_of(T value, Options ...option)
	{
		return (... || (value == option));
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T, typename ...Options>
    inline bool all_of(T value, Options ...option)
	{
		return (... && (value == option));
	}

//---------------------------------------------------------------------------------------------------------------------

}