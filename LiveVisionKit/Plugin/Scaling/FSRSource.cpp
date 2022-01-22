#include <obs/obs-module.h>
#include <obs/obs-source.h>
#include <obs/obs.h>

#include "../Scaling/FSRFilter.hpp"

//=====================================================================================
//		EVENT HANDLING
//=====================================================================================

static void* on_fsr_create(obs_data_t* settings, obs_source_t* context)
{
	auto filter = lvk::FSRFilter::Create(context);

	if(filter)
		filter->configure(settings);

	return filter;
}

//-------------------------------------------------------------------------------------

static void on_fsr_destroy(void* data)
{
	delete static_cast<lvk::FSRFilter*>(data);
}

//-------------------------------------------------------------------------------------

static void on_fsr_configure(void* data, obs_data_t* settings)
{
	static_cast<lvk::FSRFilter*>(data)->configure(settings);
}

//-------------------------------------------------------------------------------------

static void on_fsr_tick(void* data, float seconds)
{
	static_cast<lvk::FSRFilter*>(data)->update();
}

//-------------------------------------------------------------------------------------

static void on_fsr_render(void* data, gs_effect_t* _)
{
	static_cast<lvk::FSRFilter*>(data)->render();
}

//=====================================================================================
//		FILTER GETTERS
//=====================================================================================

static obs_properties_t* fsr_filter_properties(void* data)
{
	return lvk::FSRFilter::Properties();
}

//-------------------------------------------------------------------------------------

static void fsr_filter_default_settings(obs_data_t* settings)
{
	lvk::FSRFilter::LoadDefaults(settings);
}

//-------------------------------------------------------------------------------------

static uint32_t fsr_output_width(void* data)
{
	return static_cast<lvk::FSRFilter*>(data)->width();
}

//-------------------------------------------------------------------------------------

static uint32_t fsr_output_height(void* data)
{
	return static_cast<lvk::FSRFilter*>(data)->height();
}

//-------------------------------------------------------------------------------------

static const char* fsr_filter_name(void* _)
{
	return "(LVK) FidelityFX Super Resolution 1.0";
}

//=====================================================================================
//		PLUGIN REGISTRATION
//=====================================================================================

extern void register_fsr_source()
{
	obs_source_info config = {};
	config.id = "LVK~FSR";
	config.type = OBS_SOURCE_TYPE_FILTER;
	config.output_flags = OBS_SOURCE_VIDEO | OBS_SOURCE_SRGB | OBS_SOURCE_CUSTOM_DRAW;
	config.create = on_fsr_create;
	config.destroy = on_fsr_destroy;
	config.update = on_fsr_configure;
	config.video_tick = on_fsr_tick;
	config.video_render = on_fsr_render;
	config.get_name = fsr_filter_name;
	config.get_width = fsr_output_width;
	config.get_height = fsr_output_height;
	config.get_properties = fsr_filter_properties;
	config.get_defaults = fsr_filter_default_settings;

	obs_register_source(&config);
}

//-------------------------------------------------------------------------------------
