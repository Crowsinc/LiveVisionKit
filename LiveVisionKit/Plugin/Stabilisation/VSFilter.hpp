#pragma once

#include <obs/obs.h>

#include "../../LiveVisionKit.hpp"

namespace lvk
{

	class VSFilter
	{
	public:

		static obs_properties_t* Properties();

		static void LoadDefault(obs_data_t* settings);

		static VSFilter* Create(obs_source_t* context);

	public:

		void configure(obs_data_t* settings);

		void update();

		obs_source_frame* process(obs_source_frame* next_frame);

		uint32_t width() const;

		uint32_t height() const;

	private:

		obs_source_t* m_Context;

		bool m_TestMode;

		cv::Rect m_CropRegion, m_FrameRegion;
		cv::Size m_CropAmount;

		FrameTracker m_FrameTracker;
		SlidingBuffer<double> m_PathFilter;
		SlidingBuffer<Transform> m_PathWindow;
		SlidingBuffer<Transform> m_MotionQueue;

		SlidingBuffer<obs_source_frame*> m_OBSFrameQueue;
		SlidingBuffer<cv::UMat> m_StabilisationQueue;
		cv::UMat m_WarpFrame;

		VSFilter(obs_source_t* context);

		Transform clamp_to_frame(const Transform& transform);

		cv::UMat draw_test_information(const cv::UMat& test_frame);

		void update_crop_region(const cv::Size frame_size);

		void configure_sliding_windows(const uint32_t filter_radius);

		void reset_sliding_windows();

		bool validate() const;

	};

}
