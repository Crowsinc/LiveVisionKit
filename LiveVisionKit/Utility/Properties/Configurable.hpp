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

#include <functional>

namespace lvk
{

	template<typename T>
	class Configurable
	{
	public:

		// NOTE: configure() is not called on construction
		Configurable(const T& settings = {});

		virtual void configure(const T& settings = {}) = 0;

		void reconfigure(const std::function<void(T&)> updater);

		const T& settings() const;

	protected:
		T m_Settings;
	};

}

#include "Configurable.tpp"