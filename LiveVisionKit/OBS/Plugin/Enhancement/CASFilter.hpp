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

	class CASFilter
	{
	public:

		static obs_properties_t* Properties();

		static void LoadDefaults(obs_data_t* settings);

		static CASFilter* Create(obs_source_t* context, obs_data_t* settings);

		~CASFilter();

		void configure(obs_data_t* settings);

		void render();

		uint32_t width() const;

		uint32_t height() const;

	private:

		obs_source_t* m_Context;
		gs_effect_t* m_Shader;

		vec4 m_CASConst1;
		gs_eparam_t* m_CASConstParam1;
		gs_eparam_t* m_OutputSizeParam;
		vec2 m_OutputSize;


		CASFilter(obs_source_t* context);

		bool validate() const;

	};

}
