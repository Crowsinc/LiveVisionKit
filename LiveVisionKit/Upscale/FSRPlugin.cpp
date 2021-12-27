#include <obs/obs-module.h>
#include <obs/obs-source.h>
#include <obs/obs.h>

#include "FSRFilter.hpp"

/* NOTES:
 * 	- custom draw is for when you draw your own texture, otherwise the source frame
 *    is passed to the shader by OBS.
 *
 *  - async video is for accessing video data in RAM, otherwise OBS automatically
 *    passes data to the shader in a synchronous fashion.
 *
 *  - Seems like scaling occurs after all the rendering. So the FSR scaling has to be
 *    between the input and output textures. Where the input resolution is based on
 *    the source, and the output is based on the given resolution in the render call.
 *    So if the input source is 4k, and we render to 1920x1080, then the output will be
 *    1/4th the size which is then scaled to whatever the scene scaling is in the OBS output.
 */

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
	static_cast<lvk::FSRFilter*>(data)->tick();
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
	// TODO: FSRFilter::Properties();
	return NULL;
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
	//TODO: this is technically wrong because it doesn't use locale info
	return lvk::FSRFilter::Name();
}

//=====================================================================================
//		FILTER CONFIGURATION
//=====================================================================================

extern void register_fsr_filter()
{
	struct obs_source_info config = {};
	config.id = "LVK~FSR";
	config.type = OBS_SOURCE_TYPE_FILTER;
	config.icon_type = OBS_ICON_TYPE_CAMERA;
	config.output_flags = OBS_SOURCE_VIDEO; //TODO: custom draw needed?
	config.create = on_fsr_create;
	config.destroy = on_fsr_destroy;
	config.update = on_fsr_configure;
	config.video_tick = on_fsr_tick;
	config.video_render = on_fsr_render;
	config.get_name = fsr_filter_name;
	config.get_width = fsr_output_width;
	config.get_height = fsr_output_height;
	config.get_properties = fsr_filter_properties; //TODO: add .get_defaults as well?

	obs_register_source(&config);
}

//-------------------------------------------------------------------------------------
