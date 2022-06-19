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

#include "LiveVisionKit.hpp"
#include "OBS/Interop/VisionFilter.hpp"

namespace lvk
{

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

	class VSFilter : public VisionFilter
	{
	public:

		static obs_properties_t* Properties();

		static void LoadDefaults(obs_data_t* settings);

		VSFilter(obs_source_t* context);

		void tick();

		void configure(obs_data_t* settings);

		uint32_t width() const;

		uint32_t height() const;

		bool validate() const;

	private:

		virtual void filter(FrameBuffer& buffer) override;

		virtual void hybrid_render(gs_texture_t* frame) override;

		Homography clamp_velocity(const cv::UMat& frame, const Homography& velocity);

		uint64_t draw_debug_frame(
			cv::UMat& frame,
			const float tracking_stability,
			const std::vector<cv::Point2f>& trackers
		);

		void draw_debug_hud(cv::UMat& frame, const uint64_t frame_time_ns);

		bool is_queue_outdated(const FrameBuffer& new_frame) const;

		void reset_buffers();

		Homography suppress(Homography& motion);

		bool is_stabilisation_ready() const;

	private:

		obs_source_t* m_Context;

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
		
		cv::Point2f m_SuppressionRange;
		float m_SuppressionFactor;
	};

}
