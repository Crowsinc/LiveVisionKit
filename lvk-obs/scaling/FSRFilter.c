#include <obs/obs-module.h>
#include <obs/obs-source.h>
#include <obs/obs.h>


//=====================================================================================
//		EVENT HANDLING
//=====================================================================================

static void* on_fsr_create(obs_data_t* settings, obs_source_t* context)
{
	return NULL;
}

//-------------------------------------------------------------------------------------

static void on_fsr_destroy(void* data)
{

}

//-------------------------------------------------------------------------------------

static void on_fsr_configure(void* data, obs_data_t* settings)
{

}

//-------------------------------------------------------------------------------------

static void on_fsr_tick(void* data, float seconds)
{

}

//-------------------------------------------------------------------------------------

static void on_fsr_render(void* data, gs_effect_t* _)
{
//	obs_source_draw(image, x, y, cx, cy, flip)
}

//=====================================================================================
//		FILTER GETTERS
//=====================================================================================

static uint32_t fsr_output_width(void* data)
{
	return 0;
}

//-------------------------------------------------------------------------------------

static uint32_t fsr_output_height(void* data)
{
	return 0;
}

//-------------------------------------------------------------------------------------

static const char* fsr_filter_name(void* _)
{
	//TODO: this is technically wrong because it doesn't use locale info
	return "FSR Up-scaling Filter";
}


//=====================================================================================
//		FILTER CONFIGURATION
//=====================================================================================

extern void register_fsr_filter()
{
	struct obs_source_info config = {};
	config.id = "LVK-FSR";
	config.type = OBS_SOURCE_TYPE_FILTER;
	config.icon_type = OBS_ICON_TYPE_CAMERA;

	//Do we actually need custom draw?
	config.output_flags = OBS_SOURCE_VIDEO | OBS_SOURCE_CUSTOM_DRAW;
	config.create = on_fsr_create;
	config.destroy = on_fsr_destroy;
	config.update = on_fsr_configure;
	config.video_tick = on_fsr_tick;
	config.video_render = on_fsr_render;
	config.get_name = fsr_filter_name;
	config.get_width = fsr_output_width;
	config.get_height = fsr_output_height;

	obs_register_source(&config);
}


//-------------------------------------------------------------------------------------
