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
	return context;
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
	//TODO: need to buffer frames as well as UMats because we need the timestamp to sync the audio
	// See async filter to figure out how to do it properly.
	static cv::UMat buff(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);

	auto t1 = os_gettime_ns();

	buff << frame;

	auto t2 = os_gettime_ns();

	cv::imshow("OUT", buff);

	blog(LOG_INFO, "%s Extract Time: %4.2fms", get_video_format_name(frame->format), ((double)t2 - t1) * 1e-6);

	cv::putText(buff, "HAHAHA", cv::Point(100,100), cv::FONT_HERSHEY_PLAIN, 4, cv::Scalar(255, 255, 255), 4);

	t1 = os_gettime_ns();

	buff >> frame;

	t2 = os_gettime_ns();

	blog(LOG_INFO, "%s Insert Time: %4.2fms", get_video_format_name(frame->format), ((double)t2 - t1) * 1e-6);

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
