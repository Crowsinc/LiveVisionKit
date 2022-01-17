#include <obs/obs-module.h>
#include <obs/obs-source.h>
#include <obs/obs.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


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

	std::vector<cv::Mat> channels(3);


	//NOTE: dont need to copy when we do the proper thing because we just upload it up to the GPU
	channels[0] = cv::Mat(1080, 1920, CV_8UC1, frame->data[0], frame->linesize[0]);

	cv::Mat tmp;

	tmp = cv::Mat(1080/2, 1920/2, CV_8UC1, frame->data[1], frame->linesize[1]);
	cv::resize(tmp, channels[1], cv::Size(1920, 1080), 0, 0, cv::INTER_LINEAR);

	tmp = cv::Mat(1080/2, 1920/2, CV_8UC1, frame->data[2], frame->linesize[2]);
	cv::resize(tmp, channels[2], cv::Size(1920, 1080), 0, 0, cv::INTER_LINEAR);

	cv::merge(channels, tmp);

	cv::cvtColor(tmp, tmp, cv::COLOR_YUV2BGR);


	cv::imshow("TEST", tmp);

	// TODO: always output in RGBA format to avoid conversion and allow transparency
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
