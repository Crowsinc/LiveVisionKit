#pragma once

#include <obs/obs.h>
#include <opencv2/opencv.hpp>

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

		void tick();

		void render() const;

		obs_source_frame* process(obs_source_frame* obs_frame);

		void configure(obs_data_t* settings);

		uint32_t width() const;

		uint32_t height() const;

		void reset();

	private:

		struct FrameBuffer
		{
			cv::UMat frame;
			obs_source_frame* output;

			FrameBuffer();
		};

		struct FrameVector
		{
			Transform displacement;
			Transform velocity;
			uint32_t trackers;

			FrameVector(const Transform& displacement = Transform::Zero(), const Transform& velocity = Transform::Zero());

			FrameVector operator+(const FrameVector& other) const;

			FrameVector operator*(const double scaling) const;
		};

		obs_source_t* m_Context;
		gs_effect_t* m_Shader;
		gs_eparam_t* m_CropParam;

		bool m_TestMode;
		float m_CropProportion;
		uint32_t m_SmoothingRadius;

		cv::Rect m_CropRegion;
		cv::Size m_OutputSize;

		SlidingBuffer<double> m_Filter;
		SlidingBuffer<FrameVector> m_Trajectory;
		SlidingBuffer<FrameBuffer> m_FrameQueue;

		cv::UMat m_WarpFrame, m_TrackingFrame;
		FrameTracker m_FrameTracker;

		VSFilter(obs_source_t* context);

		Transform enclose_crop(const cv::UMat& frame, const Transform& transform);

		cv::UMat draw_test_mode(cv::UMat& frame, const uint64_t frame_time_ns, const uint32_t trackers);

		void reset_buffers();

		bool queue_outdated(const obs_source_frame* new_frame) const;

		bool stabilisation_ready() const;

		bool validate() const;

	};

}
