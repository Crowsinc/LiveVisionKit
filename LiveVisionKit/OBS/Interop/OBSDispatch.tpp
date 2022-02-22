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

#include <obs-module.h>

#include "Diagnostics/Directives.hpp"

namespace lvk::dispatch
{
//---------------------------------------------------------------------------------------------------------------------

	template<typename... T>
	void filter_skip(T...) {}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	obs_properties_t* filter_properties(void* data)
	{
		return T::Properties();
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	void filter_load_defaults(obs_data_t* settings)
	{
		T::LoadDefaults(settings);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	void* filter_create(obs_data_t* settings, obs_source_t* context)
	{
		return T::Create(context, settings);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	void* filter_create_auto(obs_data_t* settings, obs_source_t* context)
	{
		LVK_ASSERT(context != nullptr && settings != nullptr);

		auto filter = new T(context);

		if(!filter->validate())
		{
			delete filter;
			return nullptr;
		}

		filter->configure(settings);

		return filter;
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T, typename... O>
	void filter_delete(void* data, O...)
	{
		delete static_cast<T*>(data);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	void filter_remove(void* data, obs_source_t* parent)
	{
		static_cast<T*>(data)->terminate();
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	void filter_configure(void* data, obs_data_t* settings)
	{
		static_cast<T*>(data)->configure(settings);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	void filter_tick(void* data, float seconds)
	{
		static_cast<T*>(data)->tick();
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	void filter_render(void* data, gs_effect_t* effect)
	{
		static_cast<T*>(data)->render();
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	obs_source_frame* filter_process(void* data, obs_source_frame* frame)
	{
		return static_cast<T*>(data)->process(frame);
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	uint32_t filter_width(void* data)
	{
		return static_cast<T*>(data)->width();
	}

//---------------------------------------------------------------------------------------------------------------------

	template<typename T>
	uint32_t filter_height(void* data)
	{
		return static_cast<T*>(data)->height();
	}

//---------------------------------------------------------------------------------------------------------------------

}
