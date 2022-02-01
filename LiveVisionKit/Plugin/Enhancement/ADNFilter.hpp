#pragma once

#include <obs/obs.h>

#include <opencv2/opencv.hpp>

namespace lvk
{

	class ADNFilter
	{
	public:

		static obs_properties_t* Properties();

		static void LoadDefaults(obs_data_t* settings);

		static ADNFilter* Create(obs_source_t* context);

	public:


		void configure(obs_data_t* settings);

		obs_source_frame* process(obs_source_frame* obs_frame);

		void reset();

	private:

		obs_source_t* m_Context;

		bool m_TestMode;
		double m_Strength;
		cv::UMat m_Frame, m_SmoothFrame, m_DenoiseFrame;
		cv::UMat m_Mask, m_DetailBlendMask, m_DenoiseBlendMask;

		ADNFilter(obs_source_t* context);

		cv::UMat draw_debug_info(cv::UMat& frame, const uint64_t frame_time_ns);

		bool validate() const;

	};

}
