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

#include <obs/obs-module.h>
#include <obs/obs-source.h>

#include "CASFilter.hpp"

//---------------------------------------------------------------------------------------------------------------------

static void* on_cas_create(obs_data_t* settings, obs_source_t* context)
{
	auto filter = lvk::CASFilter::Create(context);

	if(filter)
		filter->configure(settings);

	return filter;
}

//---------------------------------------------------------------------------------------------------------------------

static void on_cas_destroy(void* data)
{
	delete static_cast<lvk::CASFilter*>(data);
}

//---------------------------------------------------------------------------------------------------------------------

static void on_cas_configure(void* data, obs_data_t* settings)
{
	static_cast<lvk::CASFilter*>(data)->configure(settings);
}

//---------------------------------------------------------------------------------------------------------------------

static void on_cas_render(void* data, gs_effect_t* effect)
{
	static_cast<lvk::CASFilter*>(data)->render();
}

//---------------------------------------------------------------------------------------------------------------------

static obs_properties_t* cas_filter_properties(void* data)
{
	return lvk::CASFilter::Properties();
}

//---------------------------------------------------------------------------------------------------------------------

static void cas_filter_default_settings(obs_data_t* settings)
{
	lvk::CASFilter::LoadDefaults(settings);
}

//---------------------------------------------------------------------------------------------------------------------

static uint32_t cas_output_width(void* data)
{
	return static_cast<lvk::CASFilter*>(data)->width();
}

//---------------------------------------------------------------------------------------------------------------------

static uint32_t cas_output_height(void* data)
{
	return static_cast<lvk::CASFilter*>(data)->height();
}

//---------------------------------------------------------------------------------------------------------------------

static const char* cas_filter_name(void* data)
{
	return "(LVK) FidelityFX Contrast Adaptive Sharpening";
}

//---------------------------------------------------------------------------------------------------------------------

extern void register_cas_source()
{
	obs_source_info config = {0};
	config.id = "LVK~CAS";
	config.type = OBS_SOURCE_TYPE_FILTER;
	config.output_flags = OBS_SOURCE_VIDEO | OBS_SOURCE_SRGB | OBS_SOURCE_CUSTOM_DRAW;

	config.create = on_cas_create;
	config.destroy = on_cas_destroy;

	config.update = on_cas_configure;
	config.video_render = on_cas_render;

	config.get_name = cas_filter_name;
	config.get_width = cas_output_width;
	config.get_height = cas_output_height;
	config.get_properties = cas_filter_properties;
	config.get_defaults = cas_filter_default_settings;

	obs_register_source(&config);
}

//---------------------------------------------------------------------------------------------------------------------
