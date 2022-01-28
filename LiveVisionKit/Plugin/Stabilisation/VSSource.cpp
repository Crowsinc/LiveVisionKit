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

static void on_vs_configure(void* data, obs_data_t* settings)
{
	static_cast<lvk::VSFilter*>(data)->configure(settings);
}

//-------------------------------------------------------------------------------------

static obs_source_frame* on_vs_async_filter(void* data, obs_source_frame* frame)
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
	config.update = on_vs_configure;
	config.filter_video = on_vs_async_filter;
	config.get_name = vs_filter_name;
	config.get_properties = vs_filter_properties;
	config.get_defaults = vs_filter_default_settings;

	obs_register_source(&config);
}
