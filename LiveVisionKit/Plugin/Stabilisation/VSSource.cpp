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

// BGR3 - GOOD - 1-2ms both ways
// RGBA - GOOD - 1-3ms both ways
// BGRA - GOOD - 1-3ms both ways
// Y800 - GOOD - < 1ms both ways
// UYVY - GOOD - 1-3ms both ways
// YUY2 - GOOD - 1-3ms both ways
// YVYU - GOOD - 1-3ms both ways
// I40A - GOOD - 1-3ms both ways
// I420 - GOOD - ~1ms both ways
// I422 - GOOD - ~2ms both ways
// I42A - GOOD - ~2ms both ways
// I444 - GOOD - ~2ms both ways
// YUVA - GOOD - ~2ms both ways
// NV12 - GOOD - ~1ms both ways
// Almost all the lag comes from the initial GPU upload/download time, which
// is made worse for uncompressed formats because there is more data to move.

static obs_source_frame* on_vs_async_filter(void* data, obs_source_frame* frame)
{
	static cv::UMat yuv(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY);
	static obs_source_frame* test_frame = nullptr;

	if(test_frame == nullptr)
		test_frame = obs_source_frame_create(video_format::VIDEO_FORMAT_RGBA, frame->width, frame->height);

	yuv << frame;

	auto t1 = os_gettime_ns();

	yuv >> test_frame;

	auto t2 = os_gettime_ns();

	blog(LOG_INFO, "%s Insert Time: %4.2fms", get_video_format_name(frame->format), ((double)t2 - t1) * 1e-6);

	t1 = os_gettime_ns();

	yuv << test_frame;

	t2 = os_gettime_ns();

	blog(LOG_INFO, "%s Extract Time: %4.2fms", get_video_format_name(frame->format), ((double)t2 - t1) * 1e-6);

	yuv >> frame;

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
