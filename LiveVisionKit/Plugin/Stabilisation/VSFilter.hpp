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

		~VSFilter();

		void configure(obs_data_t* settings);

		void tick();

		void render() const;

		obs_source_frame* process(obs_source_frame* next_frame);

		uint32_t width() const;

		uint32_t height() const;

		void reset();

	private:


		obs_source_t* m_Context;
		gs_effect_t* m_Shader;

		bool m_TestMode, m_AutoUpscale;
		float m_EdgeCropProportion;
		uint32_t m_SmoothingRadius;
		cv::Rect m_CropRegion, m_FrameRegion;
		cv::Size m_OutputSize;


		SlidingBuffer<double> m_PathFilter;
		SlidingBuffer<Transform> m_PathWindow;
		SlidingBuffer<Transform> m_MotionQueue;
		SlidingBuffer<cv::UMat> m_StabilisationQueue;
		SlidingBuffer<obs_source_frame*> m_OBSFrameQueue;

		cv::UMat m_WarpFrame, m_StabilisedFrame;

		cv::UMat m_TrackingFrame;
		FrameTracker m_FrameTracker;

		VSFilter(obs_source_t* context);

		Transform fit_to_crop(const Transform& transform);

		cv::UMat draw_test_information(cv::UMat& frame, const uint64_t ingest_time_ns, const uint64_t warp_time_ns);

		void prepare_buffers(const uint32_t filter_radius);

		void reset_buffers();

		bool stabilisation_ready() const;

		bool validate() const;

	};

}
