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

#include "ADBFilter.hpp"

//---------------------------------------------------------------------------------------------------------------------

static void* on_adb_create(obs_data_t* settings, obs_source_t* context)
{
	return lvk::ADBFilter::Create(context, settings);
}

//---------------------------------------------------------------------------------------------------------------------

static void on_adb_destroy(void* data)
{
	delete static_cast<lvk::ADBFilter*>(data);
}

//---------------------------------------------------------------------------------------------------------------------

static void on_adb_remove(void* data, obs_source_t* parent)
{
	static_cast<lvk::ADBFilter*>(data)->reset();
}

//---------------------------------------------------------------------------------------------------------------------

static void on_adb_configure(void* data, obs_data_t* settings)
{
	static_cast<lvk::ADBFilter*>(data)->configure(settings);
}

//---------------------------------------------------------------------------------------------------------------------

static obs_source_frame* on_adb_process(void* data, obs_source_frame* frame)
{
	return static_cast<lvk::ADBFilter*>(data)->process(frame);
}

//---------------------------------------------------------------------------------------------------------------------

static obs_properties_t* adb_filter_properties(void* data)
{
	return lvk::ADBFilter::Properties();
}

//---------------------------------------------------------------------------------------------------------------------

static void adb_filter_default_settings(obs_data_t* settings)
{
	lvk::ADBFilter::LoadDefaults(settings);
}

//---------------------------------------------------------------------------------------------------------------------

static const char* adb_filter_name(void* data)
{
	return "(LVK) Adapative De-Blocking";
}

//---------------------------------------------------------------------------------------------------------------------

extern void register_adb_source()
{
	obs_source_info config = {0};
	config.id = "LVK~ADB";
	config.type = OBS_SOURCE_TYPE_FILTER;
	config.output_flags = OBS_SOURCE_ASYNC_VIDEO;

	config.create = on_adb_create;
	config.destroy = on_adb_destroy;
	config.filter_remove = on_adb_remove;

	config.filter_video = on_adb_process;
	config.update = on_adb_configure;

	config.get_properties = adb_filter_properties;
	config.get_defaults = adb_filter_default_settings;
	config.get_name = adb_filter_name;

	obs_register_source(&config);
}

//---------------------------------------------------------------------------------------------------------------------
