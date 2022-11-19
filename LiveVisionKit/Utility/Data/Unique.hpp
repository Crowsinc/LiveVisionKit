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

#include <cstdint>

namespace lvk
{

	struct GlobalScope;

	template<typename Scope = GlobalScope>
	class Unique
	{
	public:

		Unique();

		Unique(const Unique& other);

		Unique(Unique&& other);

		uint64_t uid() const;
	
	private:
		static uint64_t s_NextUID;
		uint64_t m_UID;
	};

}

#include "Unique.tpp"
