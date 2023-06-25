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

#include "Directives.hpp"

namespace lvk
{
//---------------------------------------------------------------------------------------------------------------------

    template<typename T, typename P, typename R>
    CircularIterator<T,P,R>::CircularIterator(
        pointer ptr,
        const std::pair<pointer, pointer> data_bounds,
        const std::pair<pointer, pointer> access_bounds,
        const int cycle
    )
        : m_Current(ptr),
          m_Cycle(cycle),
          m_DataBounds(data_bounds),
          m_AccessBounds(access_bounds)
    {
        LVK_ASSERT(m_AccessBounds.second >= m_AccessBounds.first);
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T, typename P, typename R>
    CircularIterator<T,P,R>::pointer CircularIterator<T,P,R>::operator->()
    {
        return m_Current;
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T, typename P, typename R>
    CircularIterator<T,P,R>::reference CircularIterator<T,P,R>::operator*() const
    {
        return *m_Current;
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T, typename P, typename R>
    CircularIterator<T,P,R>& CircularIterator<T,P,R>::operator++()
    {
        if(m_Current == m_DataBounds.second)
        {
            // Cycle if we go past the final element
            m_Current = m_DataBounds.first;
            m_Cycle++;
        }
        else if(m_Current == m_AccessBounds.second)
        {
            // Wrap if we go past the right memory bound
            m_Current = m_AccessBounds.first;
        }
        else m_Current++;
        return *this;
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T, typename P, typename R>
    CircularIterator<T,P,R> CircularIterator<T,P,R>::operator++(int) const
    {
        CircularIterator<T,P,R> copy = (*this);
        ++(*this);
        return copy;
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T, typename P, typename R>
    CircularIterator<T,P,R>& CircularIterator<T,P,R>::operator--()
    {
        if(m_Current == m_DataBounds.first)
        {
            // Cycle if we go past the first element
            m_Current = m_DataBounds.second;
            m_Cycle--;
        }
        else if(m_Current == m_AccessBounds.first)
        {
            // Wrap if we go past the left memory bound
            m_Current = m_AccessBounds.second;
        }
        else m_Current--;
        return *this;
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T, typename P, typename R>
    CircularIterator<T,P,R> CircularIterator<T,P,R>::operator--(int) const
    {
        CircularIterator<T,P,R> copy = (*this);
        --(*this);
        return copy;
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T, typename P, typename R>
    bool CircularIterator<T,P,R>::operator==(const CircularIterator<T,P,R>& other) const
    {
        LVK_ASSERT(m_AccessBounds == other.m_AccessBounds && m_DataBounds == other.m_DataBounds);

        return m_Current == other.m_Current && m_Cycle == other.m_Cycle;
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T, typename P, typename R>
    bool CircularIterator<T,P,R>::operator!=(const CircularIterator<T,P,R>& other) const
    {
        LVK_ASSERT(m_AccessBounds == other.m_AccessBounds && m_DataBounds == other.m_DataBounds);

        return !operator==(other);
    }

//---------------------------------------------------------------------------------------------------------------------

    template<typename T, typename P, typename R>
    CircularIterator<T,P,R>::difference_type CircularIterator<T,P,R>::operator-(const CircularIterator& other) const
    {
        LVK_ASSERT(m_AccessBounds == other.m_AccessBounds && m_DataBounds == other.m_DataBounds);

        if(*this == other) return 0;

        const difference_type capacity = m_AccessBounds.second - m_AccessBounds.first + 1;
        const difference_type elements = (m_DataBounds.second - m_DataBounds.first + capacity) % capacity + 1;

        const difference_type idx_1 = (m_Current - m_DataBounds.first + capacity) % capacity;
        const difference_type idx_2 = (other.m_Current - m_DataBounds.first + capacity) % capacity;

        return idx_1 - idx_2 + (m_Cycle - other.m_Cycle) * elements;
    }

//---------------------------------------------------------------------------------------------------------------------


}