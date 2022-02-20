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

#include "LiveVisionKit.hpp"
#include "VSFilter.hpp"

//---------------------------------------------------------------------------------------------------------------------

static void* on_vs_create(obs_data_t* settings, obs_source_t* context)
{
	return lvk::VSFilter::Create(context, settings);
}

//---------------------------------------------------------------------------------------------------------------------

static void on_vs_destroy(void* data)
{
	delete static_cast<lvk::VSFilter*>(data);
}

//---------------------------------------------------------------------------------------------------------------------

static void on_vs_remove(void* data, obs_source_t* parent)
{
	static_cast<lvk::VSFilter*>(data)->reset();
}

//---------------------------------------------------------------------------------------------------------------------

static void on_vs_configure(void* data, obs_data_t* settings)
{
	static_cast<lvk::VSFilter*>(data)->configure(settings);
}

//---------------------------------------------------------------------------------------------------------------------

static void on_vs_tick(void* data, float seconds)
{
	static_cast<lvk::VSFilter*>(data)->tick();
}

//---------------------------------------------------------------------------------------------------------------------

static void on_vs_render(void* data, gs_effect_t* effect)
{
	static_cast<lvk::VSFilter*>(data)->render();
}

//---------------------------------------------------------------------------------------------------------------------

static obs_source_frame* on_vs_process(void* data, obs_source_frame* frame)
{
	return static_cast<lvk::VSFilter*>(data)->process(frame);
}

//---------------------------------------------------------------------------------------------------------------------

static obs_properties_t* vs_filter_properties(void* data)
{
	return lvk::VSFilter::Properties();
}

//---------------------------------------------------------------------------------------------------------------------

static void vs_filter_default_settings(obs_data_t* settings)
{
	lvk::VSFilter::LoadDefault(settings);
}

//---------------------------------------------------------------------------------------------------------------------

static uint32_t vs_output_width(void* data)
{
	return static_cast<lvk::VSFilter*>(data)->width();
}

//---------------------------------------------------------------------------------------------------------------------

static uint32_t vs_output_height(void* data)
{
	return static_cast<lvk::VSFilter*>(data)->height();
}

//---------------------------------------------------------------------------------------------------------------------

static const char* vs_filter_name(void* data)
{
	return "(LVK) Video Stabiliser";
}

//---------------------------------------------------------------------------------------------------------------------

extern void register_vs_source()
{
	obs_source_info config = {0};
	config.id = "LVK~VS";
	config.type = OBS_SOURCE_TYPE_FILTER;
	config.output_flags = OBS_SOURCE_ASYNC_VIDEO;

	config.create = on_vs_create;
	config.destroy = on_vs_destroy;
	config.filter_remove = on_vs_remove;

	config.update = on_vs_configure;
	config.video_tick = on_vs_tick;
	config.video_render = on_vs_render;
	config.filter_video = on_vs_process;

	config.get_name = vs_filter_name;
	config.get_width = vs_output_width;
	config.get_height = vs_output_height;
	config.get_properties = vs_filter_properties;
	config.get_defaults = vs_filter_default_settings;

	obs_register_source(&config);
}

//---------------------------------------------------------------------------------------------------------------------
