#include <obs/obs-module.h>
#include <obs/obs-source.h>
#include <obs/obs.h>

#include "VSFilter.hpp"

//=====================================================================================
//		EVENT HANDLING
//=====================================================================================

static void* on_vs_create(obs_data_t* settings, obs_source_t* context)
{
	auto filter = lvk::VSFilter::Create(context);

	if(filter)
		filter->configure(settings);

	return filter;
}

//-------------------------------------------------------------------------------------

static void on_vs_destroy(void* data)
{
	delete static_cast<lvk::VSFilter*>(data);
}

//-------------------------------------------------------------------------------------

static void on_vs_remove(void* data, obs_source_t* parent)
{
	static_cast<lvk::VSFilter*>(data)->reset();
}

//-------------------------------------------------------------------------------------

static void on_vs_configure(void* data, obs_data_t* settings)
{
	static_cast<lvk::VSFilter*>(data)->configure(settings);
}

//-------------------------------------------------------------------------------------

static void on_vs_tick(void* data, float seconds)
{
	static_cast<lvk::VSFilter*>(data)->tick();
}

//-------------------------------------------------------------------------------------

static void on_vs_render(void* data, gs_effect_t* _)
{
	static_cast<lvk::VSFilter*>(data)->render();
}

//-------------------------------------------------------------------------------------

static obs_source_frame* on_vs_process(void* data, obs_source_frame* frame)
{
	return static_cast<lvk::VSFilter*>(data)->process(frame);
}

//=====================================================================================
//		FILTER GETTERS
//=====================================================================================

static obs_properties_t* vs_filter_properties(void* data)
{
	return lvk::VSFilter::Properties();
}

//-------------------------------------------------------------------------------------

static void vs_filter_default_settings(obs_data_t* settings)
{
	lvk::VSFilter::LoadDefault(settings);
}

//-------------------------------------------------------------------------------------

static uint32_t vs_output_width(void* data)
{
	return static_cast<lvk::VSFilter*>(data)->width();
}

//-------------------------------------------------------------------------------------

static uint32_t vs_output_height(void* data)
{
	return static_cast<lvk::VSFilter*>(data)->height();
}

//-------------------------------------------------------------------------------------

static const char* vs_filter_name(void* _)
{
	return "(LVK) Video Stabiliser";
}

//=====================================================================================
//		PLUGIN REGISTRATION
//=====================================================================================

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
