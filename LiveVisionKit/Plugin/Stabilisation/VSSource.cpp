#include <obs/obs-module.h>
#include <obs/obs-source.h>
#include <obs/obs.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <util/platform.h>

#include "../../Vision/FrameIngest.hpp"

//=====================================================================================
//		EVENT HANDLING
//=====================================================================================

static void* on_vs_create(obs_data_t* settings, obs_source_t* context)
{
	auto i = new int;
	return i;
}

//-------------------------------------------------------------------------------------

static void on_vs_destroy(void* data)
{
	delete data;
}

//-------------------------------------------------------------------------------------

static void on_vs_configure(void* data, obs_data_t* settings)
{
}

//-------------------------------------------------------------------------------------

static void on_vs_tick(void* data, float seconds)
{
}

//-------------------------------------------------------------------------------------

static obs_source_frame* on_vs_async_filter(void* data, obs_source_frame* frame)
{
	static cv::UMat buff(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

	buff << *frame;

	cv::imshow("TEST", buff);

	// TODO: always output in RGBA format to avoid conversion and allow transparency?
	return frame;
}

//=====================================================================================
//		FILTER GETTERS
//=====================================================================================

static obs_properties_t* vs_filter_properties(void* data)
{
	return nullptr;
}

//-------------------------------------------------------------------------------------

static void vs_filter_default_settings(obs_data_t* settings)
{
}

//-------------------------------------------------------------------------------------

static uint32_t vs_output_width(void* data)
{
	return 1920;
}

//-------------------------------------------------------------------------------------

static uint32_t vs_output_height(void* data)
{
	return 1080;
}

//-------------------------------------------------------------------------------------

static const char* vs_filter_name(void* _)
{
	return "(LVK) Video Stabilisation";
}

//=====================================================================================
//		PLUGIN REGISTRATION
//=====================================================================================


extern void register_vs_source()
{
	obs_source_info config;
	config.id = "LVK~VS";
	config.type = OBS_SOURCE_TYPE_FILTER;
	config.icon_type = OBS_ICON_TYPE_CAMERA;
	config.output_flags = OBS_SOURCE_ASYNC_VIDEO; // | OBS_SOURCE_CUSTOM_DRAW
	config.create = on_vs_create;
	config.destroy = on_vs_destroy;
	config.update = on_vs_configure;
	config.video_tick = on_vs_tick;
	config.filter_video = on_vs_async_filter;
	config.get_name = vs_filter_name;
	config.get_width = vs_output_width;
	config.get_height = vs_output_height;
//	config.get_properties = vs_filter_properties;
//	config.get_defaults = vs_filter_default_settings;

	obs_register_source(&config);
}
