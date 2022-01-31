#include <obs/obs-module.h>
#include <obs/obs-source.h>
#include <obs/obs.h>

#include "CASFilter.hpp"

//=====================================================================================
//		EVENT HANDLING
//=====================================================================================

static void* on_cas_create(obs_data_t* settings, obs_source_t* context)
{
	auto filter = lvk::CASFilter::Create(context);

	if(filter)
		filter->configure(settings);

	return filter;
}

//-------------------------------------------------------------------------------------

static void on_cas_destroy(void* data)
{
	delete static_cast<lvk::CASFilter*>(data);
}

//-------------------------------------------------------------------------------------

static void on_cas_configure(void* data, obs_data_t* settings)
{
	static_cast<lvk::CASFilter*>(data)->configure(settings);
}

//-------------------------------------------------------------------------------------

static void on_cas_render(void* data, gs_effect_t* _)
{
	static_cast<lvk::CASFilter*>(data)->render();
}

//=====================================================================================
//		FILTER GETTERS
//=====================================================================================

static obs_properties_t* cas_filter_properties(void* data)
{
	return lvk::CASFilter::Properties();
}

//-------------------------------------------------------------------------------------

static void cas_filter_default_settings(obs_data_t* settings)
{
	lvk::CASFilter::LoadDefaults(settings);
}

//-------------------------------------------------------------------------------------

static uint32_t cas_output_width(void* data)
{
	return static_cast<lvk::CASFilter*>(data)->width();
}

//-------------------------------------------------------------------------------------

static uint32_t cas_output_height(void* data)
{
	return static_cast<lvk::CASFilter*>(data)->height();
}

//-------------------------------------------------------------------------------------

static const char* cas_filter_name(void* _)
{
	return "(LVK) FidelityFX Contrast Adaptive Sharpening";
}

//=====================================================================================
//		PLUGIN REGISTRATION
//=====================================================================================

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

//-------------------------------------------------------------------------------------
