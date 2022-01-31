#include "ADNFilter.hpp"

#include <obs/obs-module.h>

#include "../../Vision/FrameIngest.hpp"

namespace lvk
{

	//===================================================================================
	//		CONSTANT PROPERTIES/SETTINGS
	//===================================================================================

	constexpr auto PROP_STRENGTH = "STRENGTH";
	constexpr auto STRENGTH_MAX = 100;
	constexpr auto STRENGTH_MIN = 0;
	constexpr auto STRENGTH_DEFAULT = 25;

	//===================================================================================
	//		FILTER IMPLEMENTATION
	//===================================================================================

	obs_properties_t* ADNFilter::Properties()
	{
		obs_properties_t* properties = obs_properties_create();

		auto property = obs_properties_add_int_slider(
			properties,
			PROP_STRENGTH,
			"Strength",
			STRENGTH_MIN,
			STRENGTH_MAX,
			1
		);
		obs_property_int_set_suffix(property, "%");

		return properties;
	}

	//-------------------------------------------------------------------------------------

	void ADNFilter::LoadDefaults(obs_data_t* settings)
	{
		obs_data_set_default_int(settings, PROP_STRENGTH, STRENGTH_DEFAULT);
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
		  m_Strength(0),
		  m_Frame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_SmoothFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_DenoiseFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_Mask(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_DetailBlendMask(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_DenoiseBlendMask(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY)
	{}

	//-------------------------------------------------------------------------------------

	void ADNFilter::configure(obs_data_t* settings)
	{
		m_Strength = obs_data_get_int(settings, PROP_STRENGTH) / 100.0f;
	}

	//-------------------------------------------------------------------------------------

	obs_source_frame* ADNFilter::process(obs_source_frame* obs_frame)
	{
		m_Frame << obs_frame;
		cv::extractChannel(m_Frame, m_Mask, 0);
		cv::cvtColor(m_Frame, m_Frame, cv::COLOR_YUV2BGR);

		// Perform denoising
		const cv::Size denoise_resolution(480, 270);
		cv::resize(m_Frame, m_DenoiseFrame, denoise_resolution, 0, 0, cv::INTER_AREA);
		cv::fastNlMeansDenoising(m_DenoiseFrame, m_DenoiseFrame, 5, 7, 15);
		cv::resize(m_DenoiseFrame, m_SmoothFrame, m_Frame.size(), 0, 0, cv::INTER_LINEAR);

		// Create detail and non-detail masks

		const int threshold = m_Strength * 40;

		cv::Sobel(m_Mask, m_Mask, m_Mask.type(), 1, 1);
		cv::equalizeHist(m_Mask, m_Mask);

		cv::boxFilter(m_Mask, m_Mask, m_Mask.type(), cv::Size(11,11));
		cv::threshold(m_Mask, m_Mask, threshold, 255, cv::THRESH_BINARY);
		cv::boxFilter(m_Mask, m_Mask, m_Mask.type(), cv::Size(21,21));

		m_Mask.convertTo(m_DetailBlendMask, CV_32FC1, 1.0/255);
		cv::bitwise_not(m_Mask, m_Mask);
		m_Mask.convertTo(m_DenoiseBlendMask, CV_32FC1, 1.0/255);

		// Blend denoised and original frame
		cv::blendLinear(m_Frame, m_SmoothFrame, m_DetailBlendMask, m_DenoiseBlendMask, m_Frame);

		cv::cvtColor(m_Frame, m_Frame, cv::COLOR_BGR2YUV);
		m_Frame >> obs_frame;

		return obs_frame;
	}

	//-------------------------------------------------------------------------------------

	void ADNFilter::reset()
	{
		// Release all GPU memory
		m_Frame.release();
		m_SmoothFrame.release();
		m_DenoiseFrame.release();

		m_Mask.release();
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
