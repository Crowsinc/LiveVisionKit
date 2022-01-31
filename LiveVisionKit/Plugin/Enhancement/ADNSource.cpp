#include <obs/obs-module.h>
#include <obs/obs-source.h>
#include <obs/obs.h>

#include "ADNFilter.hpp"

//=====================================================================================
//		EVENT HANDLING
//=====================================================================================

static void* on_adn_create(obs_data_t* settings, obs_source_t* context)
{
	auto filter = lvk::ADNFilter::Create(context);

	if(filter)
		filter->configure(settings);

	return filter;
}

//-------------------------------------------------------------------------------------

static void on_adn_destroy(void* data)
{
	delete static_cast<lvk::ADNFilter*>(data);
}

//-------------------------------------------------------------------------------------

static void on_adn_remove(void* data, obs_source_t* parent)
{
	static_cast<lvk::ADNFilter*>(data)->reset();
}

//-------------------------------------------------------------------------------------

static void on_adn_configure(void* data, obs_data_t* settings)
{
	static_cast<lvk::ADNFilter*>(data)->configure(settings);
}

//-------------------------------------------------------------------------------------

static obs_source_frame* on_adn_process(void* data, obs_source_frame* frame)
{
	return static_cast<lvk::ADNFilter*>(data)->process(frame);
}

//=====================================================================================
//		FILTER GETTERS
//=====================================================================================

static obs_properties_t* adn_filter_properties(void* data)
{
	return lvk::ADNFilter::Properties();
}

//-------------------------------------------------------------------------------------

static void adn_filter_default_settings(obs_data_t* settings)
{
	lvk::ADNFilter::LoadDefaults(settings);
}

//-------------------------------------------------------------------------------------

static const char* adn_filter_name(void* _)
{
	return "(LVK) Adapative Denoiser";
}

//=====================================================================================
//		PLUGIN REGISTRATION
//=====================================================================================

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

//-------------------------------------------------------------------------------------
