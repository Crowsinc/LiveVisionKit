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

#include "Unique.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	template<typename Scope>
	uint64_t Unique<Scope>::s_NextUID = 1;

//---------------------------------------------------------------------------------------------------------------------

	template<typename Scope>
	inline Unique<Scope>::Unique()
		: m_UID(s_NextUID++)
	{}
	
//---------------------------------------------------------------------------------------------------------------------

	template<typename Scope>
	inline Unique<Scope>::Unique(const Unique& other)
		: m_UID(s_NextUID++)
	{}

//---------------------------------------------------------------------------------------------------------------------

	template<typename Scope>
	inline Unique<Scope>::Unique(Unique&& other)
		: m_UID(other.m_UID)
	{}

//---------------------------------------------------------------------------------------------------------------------

	template<typename Scope>
	inline uint64_t Unique<Scope>::uid() const
	{
		return m_UID;
	}

//---------------------------------------------------------------------------------------------------------------------

}
