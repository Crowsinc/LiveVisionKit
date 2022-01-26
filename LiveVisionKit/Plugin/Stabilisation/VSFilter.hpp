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

		void configure(obs_data_t* settings);

		obs_source_frame* process(obs_source_frame* obs_frame);

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

			FrameVector(const Transform& displacement = Transform::Zero(), const Transform& velocity = Transform::Zero());

			FrameVector operator+(const FrameVector& other) const;

			FrameVector operator*(const double scaling) const;
		};

		obs_source_t* m_Context;
		uint32_t m_SmoothingRadius;
		float m_EdgeCropProportion;
		bool m_RemakeBuffers;
		bool m_TestMode;

		SlidingBuffer<double> m_Filter;
		SlidingBuffer<FrameVector> m_Trajectory;
		SlidingBuffer<FrameBuffer> m_FrameQueue;

		cv::UMat m_WarpFrame;
		cv::UMat m_TrackingFrame;
		cv::UMat m_StabilisedFrame;
		FrameTracker m_FrameTracker;

		VSFilter(obs_source_t* context);

		cv::Rect find_crop_region(const cv::UMat& frame);

		Transform respect_crop(const cv::UMat& frame, const Transform& transform, const cv::Rect& crop_region);

		cv::UMat draw_test_mode(cv::UMat& frame, const cv::Rect& crop, const uint64_t frame_time_ns);

		void prepare_buffers();

		void reset_buffers();

		bool stabilisation_ready() const;

		bool validate() const;

		Transform filter(SlidingBuffer<FrameVector>& motions) const;
	};

}
