//    *************************** LiveVisionKit ****************************
//    Copyright (C) 2022  Sebastian Di Marco (crowsinc.dev@gmail.com)
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.
// 	  **********************************************************************

#pragma once

#include "../../LiveVisionKit.hpp"

namespace lvk
{

	class VSFilter
	{
	public:

		static obs_properties_t* Properties();

		static void LoadDefault(obs_data_t* settings);

		static VSFilter* Create(obs_source_t* context, obs_data_t* settings);

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
			Homography displacement;
			Homography velocity;

			FrameVector(
				const Homography& displacement = Homography::Zero(),
				const Homography& velocity = Homography::Zero()
			);

			FrameVector operator+(const FrameVector& other) const;

			FrameVector operator-(const FrameVector& other) const;

			FrameVector operator*(const double scaling) const;

			FrameVector operator/(const double scaling) const;

		};

		obs_source_t* m_Context;

		gs_effect_t* m_Shader;
		gs_eparam_t* m_CropParam;

		bool m_Enabled, m_TestMode;
		uint32_t m_SmoothingRadius;
		float m_CropProportion;

		cv::Rect m_CropRegion;
		cv::Size m_OutputSize;

		SlidingBuffer<double> m_Filter;
		SlidingBuffer<FrameVector> m_Trajectory;
		SlidingBuffer<FrameBuffer> m_FrameQueue;

		cv::UMat m_WarpFrame, m_TrackingFrame;
		FrameTracker m_FrameTracker;


		VSFilter(obs_source_t* context);

		void reset_buffers();

		Homography clamp_velocity(const cv::UMat& frame, const Homography& velocity);

		uint64_t draw_debug_frame(cv::UMat& frame, const std::vector<cv::Point2f>& trackers);

		cv::UMat draw_debug_hud(cv::UMat& frame, const uint64_t frame_time_ns);

		bool is_queue_outdated(const obs_source_frame* new_frame) const;

		void release_frame_queue();

		bool stabilisation_ready() const;

		bool validate() const;

	};

}
