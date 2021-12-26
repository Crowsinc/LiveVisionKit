#include <obs/obs-module.h>
#include <obs/obs-source.h>
#include <obs/obs.h>

#include <string>

//TODO: move to FSRShader.hpp class (and change this file to .c)
#define A_CPU 1
#include "effects/ffx_a.h"
#include "effects/ffx_fsr1.h"

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
//		FILTER STRUCTURE
//=====================================================================================

//TODO: make FSRShader class which is responsible for setting up constants
// loading, and storing the gs_effect_t handle.
struct FSRConstants
{
	// FSR EASU constants
	varAU4(easu_const_0);
	varAU4(easu_const_1);
	varAU4(easu_const_2);
	varAU4(easu_const_3);

	// FSR RCAS constants
	varAU4(rcas_const_0);
};

//TODO: ensure we compile with C++ so we don't have to the typedef.
struct FSRFilterData
{
	obs_source_t* context;

	gs_effect_t* shader;
	FSRConstants shader_params;
};

//=====================================================================================
//		EVENT HANDLING
//=====================================================================================

static void* on_fsr_create(obs_data_t* settings, obs_source_t* context)
{
	FSRFilterData* filter = static_cast<FSRFilterData*>(bzalloc(sizeof(FSRFilterData)));

	// Initialize filter data structure
	filter->context = context;


	// Load necessary shaders

	//TODO: make this use the module load macro
	const char* effect_path = "/home/sdm/Projects/C++/LiveVisionKit/lvk-obs/scaling/effects/fsr.effect";
//	const char* effect_path = obs_module_file("fsr.effect");

	obs_enter_graphics();

	filter->shader = gs_effect_create_from_file(effect_path, NULL);

	obs_leave_graphics();

//	bfree(effect_path);

	if(filter->shader == NULL)
	{
		blog(LOG_ERROR, "Couldn't load shader!\n");
		bfree(filter);
		return NULL;
	}

	return filter;
}

//-------------------------------------------------------------------------------------

static void on_fsr_destroy(void* data)
{
	bfree(data);
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
	FSRFilterData* filter = static_cast<FSRFilterData*>(data);

	if(!obs_source_process_filter_begin(filter->context, GS_RGBA, OBS_ALLOW_DIRECT_RENDERING))
	{
		blog(LOG_ERROR, "Failed to start render!\n");
		return;
	}


	// This eventually draws the frame as a sprite of the width and height passed
	// to this function. So if the original texture is less than the size. So the
	// output of the shader is the given resolution. If the sprite's texture is
	// less than this resolution, then the texture sampler will have to perform
	// the filtering required to up-sample the coordinates. So basically, we should
	// be able to just set the output res here and sample the texture as normal
	// with linear filtering?
	obs_source_process_filter_end(filter->context, filter->shader, 1920, 1080);
}

//=====================================================================================
//		FILTER GETTERS
//=====================================================================================

static obs_properties_t* fsr_filter_properties(void* data)
{
	// Configuration of the settings GUI
	return NULL;
}

//-------------------------------------------------------------------------------------

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
	return "LVK - FSR Upscaler";
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
