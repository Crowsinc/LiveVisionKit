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

#include "ADNFilter.hpp"

//---------------------------------------------------------------------------------------------------------------------

static void* on_adn_create(obs_data_t* settings, obs_source_t* context)
{
	return lvk::ADNFilter::Create(context, settings);
}

//---------------------------------------------------------------------------------------------------------------------

static void on_adn_destroy(void* data)
{
	delete static_cast<lvk::ADNFilter*>(data);
}

//---------------------------------------------------------------------------------------------------------------------

static void on_adn_remove(void* data, obs_source_t* parent)
{
	static_cast<lvk::ADNFilter*>(data)->reset();
}

//---------------------------------------------------------------------------------------------------------------------

static void on_adn_configure(void* data, obs_data_t* settings)
{
	static_cast<lvk::ADNFilter*>(data)->configure(settings);
}

//---------------------------------------------------------------------------------------------------------------------

static obs_source_frame* on_adn_process(void* data, obs_source_frame* frame)
{
	return static_cast<lvk::ADNFilter*>(data)->process(frame);
}

//---------------------------------------------------------------------------------------------------------------------

static obs_properties_t* adn_filter_properties(void* data)
{
	return lvk::ADNFilter::Properties();
}

//---------------------------------------------------------------------------------------------------------------------

static void adn_filter_default_settings(obs_data_t* settings)
{
	lvk::ADNFilter::LoadDefaults(settings);
}

//---------------------------------------------------------------------------------------------------------------------

static const char* adn_filter_name(void* data)
{
	return "(LVK) Adapative Denoiser";
}

//---------------------------------------------------------------------------------------------------------------------

extern void register_adn_source()
{
	obs_source_info config = {0};
	config.id = "LVK~ADN";
	config.type = OBS_SOURCE_TYPE_FILTER;
	config.output_flags = OBS_SOURCE_ASYNC_VIDEO;

	config.create = on_adn_create;
	config.destroy = on_adn_destroy;
	config.filter_remove = on_adn_remove;

	config.filter_video = on_adn_process;
	config.update = on_adn_configure;

	config.get_properties = adn_filter_properties;
	config.get_defaults = adn_filter_default_settings;
	config.get_name = adn_filter_name;

	obs_register_source(&config);
}

//---------------------------------------------------------------------------------------------------------------------
