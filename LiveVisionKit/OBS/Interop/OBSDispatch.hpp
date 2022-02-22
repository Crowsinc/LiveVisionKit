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

#include <obs-module.h>

namespace lvk::dispatch
{

	// Automatic creation of OBS function pointers for the dispatch
	// of filter (source) operations. The filter itself must implement
	// each of the functions called by the dispatch operation in the .tpp

	template<typename... T>
	void skip(T...);


	template<typename T>
	obs_properties_t* filter_properties(void* data);


	template<typename T>
	void filter_load_defaults(obs_data_t* settings);


	template<typename T>
	void* filter_create(obs_data_t* settings, obs_source_t* context);


	template<typename T>
	void* filter_create_auto(obs_data_t* settings, obs_source_t* context);


	template<typename T, typename... O>
	void filter_delete(void* data, O...);


	template<typename T>
	void filter_remove(void* data, obs_source_t* parent);


	template<typename T>
	void filter_configure(void* data, obs_data_t* settings);


	template<typename T>
	void filter_tick(void* data, float seconds);


	template<typename T>
	void filter_render(void* data, gs_effect_t* effect);


	template<typename T>
	obs_source_frame* filter_process(void* data, obs_source_frame* frame);


	template<typename T>
	uint32_t filter_width(void* data);


	template<typename T>
	uint32_t filter_height(void* data);

}

#include "OBSDispatch.tpp"
