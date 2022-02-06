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
#include <obs-source.h>
#include <obs.h>

#include "FSRFilter.hpp"

//---------------------------------------------------------------------------------------------------------------------

static void* on_fsr_create(obs_data_t* settings, obs_source_t* context)
{
	return lvk::FSRFilter::Create(context, settings);
}

//---------------------------------------------------------------------------------------------------------------------

static void on_fsr_destroy(void* data)
{
	delete static_cast<lvk::FSRFilter*>(data);
}

//---------------------------------------------------------------------------------------------------------------------

static void on_fsr_configure(void* data, obs_data_t* settings)
{
	static_cast<lvk::FSRFilter*>(data)->configure(settings);
}

//---------------------------------------------------------------------------------------------------------------------

static void on_fsr_render(void* data, gs_effect_t* effect)
{
	static_cast<lvk::FSRFilter*>(data)->render();
}

//---------------------------------------------------------------------------------------------------------------------

static obs_properties_t* fsr_filter_properties(void* data)
{
	return lvk::FSRFilter::Properties();
}

//---------------------------------------------------------------------------------------------------------------------

static void fsr_filter_default_settings(obs_data_t* settings)
{
	lvk::FSRFilter::LoadDefaults(settings);
}

//---------------------------------------------------------------------------------------------------------------------

static uint32_t fsr_output_width(void* data)
{
	return static_cast<lvk::FSRFilter*>(data)->width();
}

//---------------------------------------------------------------------------------------------------------------------

static uint32_t fsr_output_height(void* data)
{
	return static_cast<lvk::FSRFilter*>(data)->height();
}

//---------------------------------------------------------------------------------------------------------------------

static const char* fsr_filter_name(void* data)
{
	return "(LVK) FidelityFX Super Resolution 1.0";
}

//---------------------------------------------------------------------------------------------------------------------

extern void register_fsr_source()
{
	obs_source_info config = {0};
	config.id = "LVK~FSR";
	config.type = OBS_SOURCE_TYPE_FILTER;
	config.output_flags = OBS_SOURCE_VIDEO | OBS_SOURCE_SRGB | OBS_SOURCE_CUSTOM_DRAW;

	config.create = on_fsr_create;
	config.destroy = on_fsr_destroy;

	config.update = on_fsr_configure;
	config.video_render = on_fsr_render;

	config.get_name = fsr_filter_name;
	config.get_width = fsr_output_width;
	config.get_height = fsr_output_height;
	config.get_properties = fsr_filter_properties;
	config.get_defaults = fsr_filter_default_settings;

	obs_register_source(&config);
}

//---------------------------------------------------------------------------------------------------------------------
