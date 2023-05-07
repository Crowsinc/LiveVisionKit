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

#include "ScopedProfiler.hpp"

#include <obs.h>

namespace lvk
{
//---------------------------------------------------------------------------------------------------------------------

    ScopedProfiler::ScopedProfiler(const char* scope_name)
        : m_ScopeName(scope_name)
    {
        profile_start(scope_name);
    }

//---------------------------------------------------------------------------------------------------------------------

    ScopedProfiler::~ScopedProfiler()
    {
        profile_end(m_ScopeName);
    }

//---------------------------------------------------------------------------------------------------------------------
}