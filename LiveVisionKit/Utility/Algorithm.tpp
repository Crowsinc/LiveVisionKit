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

#include <algorithm>

#include "../Diagnostics/Assert.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	void fast_erase(std::vector<T>& data, const size_t index)
	{
		LVK_ASSERT(index < data.size());

		// Removes an element by first swapping it with the final element to avoid re-shuffling.
		std::swap(data[index], data.back());
		data.pop_back();
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T, typename P>
	void fast_filter(std::vector<T>& data, const std::vector<P>& keep, bool invert)
	{
		LVK_ASSERT(data.size() == keep.size());

		// We need to filter in reverse so that the fast erase doesn't affect the
		// data/keep element correspondence of unprocessed elements.
		for(int k = keep.size() - 1; k >= 0; k--)
			if(invert == static_cast<bool>(keep[k]))
				fast_erase(data, k);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T, typename P>
	void fast_filter(std::vector<T>& data_1, std::vector<T>& data_2, const std::vector<P>& keep, bool invert)
	{
		LVK_ASSERT(data_1.size() == keep.size());
		LVK_ASSERT(data_2.size() == keep.size());

		// We need to filter in reverse so that the fast erase doesn't affect the
		// vector element correspondence of unprocessed elements.
		for(int k = keep.size() - 1; k >= 0; k--)
			if(invert == static_cast<bool>(keep[k]))
			{
				fast_erase(data_1, k);
				fast_erase(data_2, k);
			}
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T, typename P>
	void filter(std::vector<T>& data, const std::vector<P>& keep, bool invert)
	{
		LVK_ASSERT(data.size() == keep.size());

		// https://stackoverflow.com/a/33494518
		const auto predicate = [&](const T& value) {
			return invert == static_cast<bool>(keep.at(&value - &data[0]));
		};

		data.erase(std::remove_if(data.begin(), data.end(), predicate), data.end());
	}

//---------------------------------------------------------------------------------------------------------------------

}
