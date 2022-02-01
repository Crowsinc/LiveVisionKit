#include "ADNFilter.hpp"

#include <obs/obs-module.h>

#include <util/platform.h>

#include "../../Vision/FrameIngest.hpp"

namespace lvk
{

	//===================================================================================
	//		CONSTANT PROPERTIES/SETTINGS
	//===================================================================================

	constexpr auto PROP_STRENGTH = "STRENGTH";
	constexpr auto STRENGTH_MAX = 10;
	constexpr auto STRENGTH_MIN = 0;
	constexpr auto STRENGTH_DEFAULT = 5;

	static constexpr auto PROP_TEST_MODE = "TEST_MODE";
	static constexpr auto TEST_MODE_DEFAULT = false;

	//===================================================================================
	//		FILTER IMPLEMENTATION
	//===================================================================================

	obs_properties_t* ADNFilter::Properties()
	{
		obs_properties_t* properties = obs_properties_create();

		obs_properties_add_int_slider(
			properties,
			PROP_STRENGTH,
			"Strength",
			STRENGTH_MIN,
			STRENGTH_MAX,
			1
		);

		obs_properties_add_bool(
				properties,
				PROP_TEST_MODE,
				"Test Mode"
		);

		return properties;
	}

	//-------------------------------------------------------------------------------------

	void ADNFilter::LoadDefaults(obs_data_t* settings)
	{
		obs_data_set_default_int(settings, PROP_STRENGTH, STRENGTH_DEFAULT);
		obs_data_set_default_bool(settings, PROP_TEST_MODE, TEST_MODE_DEFAULT);
	}

	//-------------------------------------------------------------------------------------

	ADNFilter* ADNFilter::Create(obs_source_t* context)
	{
		auto filter = new ADNFilter(context);

		if(!filter->validate())
		{
			delete filter;
			return nullptr;
		}

		return filter;
	}

	//-------------------------------------------------------------------------------------

	ADNFilter::ADNFilter(obs_source_t* context)
		: m_Context(context),
		  m_TestMode(false),
		  m_Strength(0),
		  m_Frame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_SmoothFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_DenoiseFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_Edges(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_Mask(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_DetailBlendMask(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_DenoiseBlendMask(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY)
	{}

	//-------------------------------------------------------------------------------------

	void ADNFilter::configure(obs_data_t* settings)
	{
		m_Strength = obs_data_get_int(settings, PROP_STRENGTH) / 100.0f;
		m_TestMode = obs_data_get_bool(settings, PROP_TEST_MODE);
	}

	//-------------------------------------------------------------------------------------

	obs_source_frame* ADNFilter::process(obs_source_frame* obs_frame)
	{
		const auto start_time = os_gettime_ns();

		m_Frame << obs_frame;

		cv::extractChannel(m_Frame, m_Mask, 0);
		cv::cvtColor(m_Frame, m_Frame, cv::COLOR_YUV2BGR);

		// Perform denoising
		const cv::Size denoise_resolution(480, 270);
		cv::resize(m_Frame, m_DenoiseFrame, denoise_resolution, 0, 0, cv::INTER_AREA);
		cv::medianBlur(m_DenoiseFrame, m_DenoiseFrame, 5);
		cv::resize(m_DenoiseFrame, m_SmoothFrame, m_Frame.size(), 0, 0, cv::INTER_LINEAR);

		// Create detail mask
		cv::Sobel(m_Mask, m_Edges, m_Mask.type(), 1, 0);
		cv::Sobel(m_Mask, m_Mask, m_Mask.type(), 0, 1);
		cv::bitwise_or(m_Mask, m_Edges, m_Mask);
		cv::boxFilter(m_Mask, m_Mask, m_Mask.type(), cv::Size(21, 21));

		const int threshold = m_Strength * 80;
		cv::threshold(m_Mask, m_Mask, threshold, 255, cv::THRESH_BINARY);

		// Add fall-off and linear blend
		cv::boxFilter(m_Mask, m_Mask, m_Mask.type(), cv::Size(11, 11));
		m_Mask.convertTo(m_DetailBlendMask, CV_32FC1, 1.0/255);
		cv::bitwise_not(m_Mask, m_Mask);
		m_Mask.convertTo(m_DenoiseBlendMask, CV_32FC1, 1.0/255);

		if(m_TestMode)
			m_SmoothFrame.setTo(cv::Scalar(255, 0, 255));

		// Blend denoised and original frame
		cv::blendLinear(m_Frame, m_SmoothFrame, m_DetailBlendMask, m_DenoiseBlendMask, m_Frame);

		cv::cvtColor(m_Frame, m_Frame, cv::COLOR_BGR2YUV);

		m_Frame >> obs_frame;

		const auto end_time = os_gettime_ns();

		if(m_TestMode)
			draw_debug_info(m_Frame, end_time - start_time) >> obs_frame;

		return obs_frame;
	}

	//-------------------------------------------------------------------------------------

	cv::UMat ADNFilter::draw_debug_info(cv::UMat& frame, const uint64_t frame_time_ns)
	{
		const cv::Scalar magenta_yuv(105, 212, 234);
		const cv::Scalar green_yuv(149, 43, 21);
		const cv::Scalar red_yuv(76, 84, 255);

		const double bad_time_threshold_ms = 5.0;
		const double frame_time_ms = frame_time_ns * 1.0e-6;

		//TODO: switch to C++20 fmt as soon as GCC supports it.
		std::stringstream time_text;
		time_text << std::fixed << std::setprecision(2);
		time_text << frame_time_ms << "ms";

		cv::putText(
			frame,
			time_text.str(),
			cv::Point(5, 40),
			cv::FONT_HERSHEY_DUPLEX,
			1.5,
			frame_time_ms < bad_time_threshold_ms ? green_yuv : red_yuv,
			2
		);

		return frame;
	}

	//-------------------------------------------------------------------------------------

	void ADNFilter::reset()
	{
		// Release all GPU memory
		m_Frame.release();
		m_SmoothFrame.release();
		m_DenoiseFrame.release();

		m_Mask.release();
		m_Edges.release();
		m_DetailBlendMask.release();
		m_DenoiseBlendMask.release();
	}

	//-------------------------------------------------------------------------------------

	bool ADNFilter::validate() const
	{
		return m_Context != nullptr;
	}

	//-------------------------------------------------------------------------------------
}
