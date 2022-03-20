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

#include "LiveVisionKit.hpp"

namespace lvk
{

	//TODO: Build proper shader API for the purposes of easily creating & re-using effects in various filters.

	template<typename E>
	class OBSEffect
	{
	public:

		static E& Get();

		static bool Validate();

	protected:

		OBSEffect(const std::string& name);

		~OBSEffect();

		gs_effect_t* handle();

		virtual bool validate() const = 0;

	private:
		gs_effect_t* m_Handle;
	};

}


#include "OBSEffect.tpp"
