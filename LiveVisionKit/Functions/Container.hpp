//     *************************** LiveVisionKit ****************************
//     Copyright (C) 2022  Sebastian Di Marco (crowsinc.dev@gmail.com)
//
//     This program is free software: you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation, either version 3 of the License, or
//     (at your option) any later version.
//
//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.
//
//     You should have received a copy of the GNU General Public License
//     along with this program.  If not, see <https://www.gnu.org/licenses/>.
//     **********************************************************************

#pragma once

#include <vector>
#include <string>
#include <algorithm>
#include <functional>

namespace lvk
{
	// Erase quickly without care for preserving the element ordering
	template<typename T>
	void fast_erase(std::vector<T>& data, const size_t index);

	template<typename T, typename P>
	void filter(std::vector<T>& data, const std::vector<P>& keep, bool invert = false);

	// NOTE: does not preserve element ordering
	template<typename T, typename P>
	void fast_filter(std::vector<T>& data, const std::vector<P>& keep, bool invert = false);

    // NOTE: does not preserve element ordering
	template<typename T1, typename T2, typename P>
	void fast_filter(
        std::vector<T1>& data_1,
        std::vector<T2>& data_2,
        const std::vector<P>& keep,
        bool invert = false
    );

    // NOTE: does not preserve element ordering
    template<typename T1, typename T2, typename T3, typename P>
    void fast_filter(
        std::vector<T1>& data_1,
        std::vector<T2>& data_2,
        std::vector<T3>& data_3,
        const std::vector<P>& keep,
        bool invert = false
    );





    template<typename T>
    float ratio_of(const std::vector<T>& data, const T& value);


    template<typename iterator>
    auto max(const iterator begin, const iterator end);

    template<typename iterator>
    auto min(const iterator begin, const iterator end);

    template<typename iterator>
    auto sum(const iterator begin, const iterator end);

    template<typename iterator>
    auto mean(const iterator begin, const iterator end);

    template<typename iterator>
    auto variance(const iterator begin, const iterator end);

}

#include "Container.tpp"

