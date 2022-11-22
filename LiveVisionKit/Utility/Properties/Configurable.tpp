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

#include "Configurable.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	inline Configurable<T>::Configurable(const T& settings)
		: m_Settings(settings)
	{}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	inline void Configurable<T>::reconfigure(const std::function<void(T&)> updater)
	{
		T new_settings = settings();
		updater(new_settings);
		configure(new_settings);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	inline const T& Configurable<T>::settings() const
	{
		return m_Settings;
	}

//---------------------------------------------------------------------------------------------------------------------

}