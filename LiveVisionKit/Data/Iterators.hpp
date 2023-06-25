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

#include <iterator>

namespace lvk
{




    template<typename T, typename P, typename R>
    struct CircularIterator
    {
        using value_type = T; using pointer = P; using reference = R;
        using iterator_category = std::bidirectional_iterator_tag;
        using difference_type = std::ptrdiff_t;

        // NOTE: bounds represent accessible memory pointers.
        CircularIterator(
            pointer ptr,
            const std::pair<pointer, pointer> data_bounds,
            const std::pair<pointer, pointer> access_bounds,
            const int cycle = 0
        );

        pointer operator->();
        reference operator*() const;

        CircularIterator& operator++();
        CircularIterator operator++(int) const;

        CircularIterator& operator--();
        CircularIterator operator--(int) const;

        bool operator==(const CircularIterator&) const;
        bool operator!=(const CircularIterator&) const;

        difference_type operator-(const CircularIterator&) const;

    private:
        const std::pair<pointer, pointer> m_DataBounds;
        const std::pair<pointer, pointer> m_AccessBounds;

        pointer m_Current;
        int m_Cycle = 0;
    };

    template<typename T>
    using circular_iterator = CircularIterator<T, T*, T&>;

    template<typename T>
    using const_circular_iterator = CircularIterator<const T, const T*, const T&>;
}


#include "Iterators.tpp"