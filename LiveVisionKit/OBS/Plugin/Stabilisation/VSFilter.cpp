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

#include "VSFilter.hpp"

#include <sstream>
#include <util/platform.h>

#include "OBS/Effects/FSREffect.hpp"
#include "OBS/Effects/DefaultEffect.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	constexpr auto PROP_SMOOTHING_RADIUS = "SMOOTH_RADIUS";
	constexpr auto SMOOTHING_RADIUS_DEFAULT = 10;
	constexpr auto SMOOTHING_RADIUS_MIN = 2;
	constexpr auto SMOOTHING_RADIUS_MAX = 20;

	constexpr auto PROP_STREAM_DELAY_INFO = "STREAM_DELAY_INFO";
	constexpr auto STREAM_DELAY_INFO_MIN = 0;
	constexpr auto STREAM_DELAY_INFO_MAX = 100 * SMOOTHING_RADIUS_MAX;

	constexpr auto PROP_CROP_PERCENTAGE = "CROP_PERCENTAGE";
	constexpr auto CROP_PERCENTAGE_DEFAULT = 5;
	constexpr auto CROP_PERCENTAGE_MIN = 1;
	constexpr auto CROP_PERCENTAGE_MAX = 25;

	constexpr auto PROP_MOTION_MODEL = "MOTION_MODEL";
	constexpr auto MOTION_MODEL_AFFINE = "AFFINE";
	constexpr auto MOTION_MODEL_HOMOGRAPHY = "HOMOGRAPHY";
	
	constexpr auto MOTION_MODEL_DYNAMIC = "DYNAMIC";
	constexpr auto MOTION_MODEL_DEFAULT = MOTION_MODEL_DYNAMIC;

	constexpr auto PROP_SUPPRESSION_MODE = "SUPPRESSION_MODE";
	constexpr auto SUPPRESSION_MODE_OFF = "SM_OFF";
	constexpr auto SUPPRESSION_MODE_STRICT = "SM_STRICT";
	constexpr auto SUPPRESSION_MODE_RELAXED = "SM_RELAXED";
	constexpr auto SUPPRESSION_MODE_DEFAULT = SUPPRESSION_MODE_RELAXED;

	const auto SUPPRESSION_RANGE_OFF = cv::Point2f(0.0f, 0.0f);
	const auto SUPPRESSION_RANGE_STRICT = cv::Point2f(0.70f, 0.90f);
	const auto SUPPRESSION_RANGE_RELAXED = cv::Point2f(0.0f, 0.30f);
	constexpr auto SUPPRESSION_SMOOTHING_STEP = 3.0f;

	constexpr auto PROP_STAB_DISABLED = "STAB_DISABLED";
	constexpr auto STAB_DISABLED_DEFAULT = false;

	constexpr auto PROP_TEST_MODE = "TEST_MODE";
	constexpr auto TEST_MODE_DEFAULT = false;

	constexpr auto TIMING_THRESHOLD_MS = 6.0;

//---------------------------------------------------------------------------------------------------------------------

	obs_properties_t* VSFilter::Properties()
	{
		obs_properties_t* properties = obs_properties_create();

		auto property = obs_properties_add_int(
			properties,
			PROP_SMOOTHING_RADIUS,
			"Smoothing Radius",
			SMOOTHING_RADIUS_MIN,
			SMOOTHING_RADIUS_MAX,
			2
		);

		property = obs_properties_add_int(
			properties,
			PROP_STREAM_DELAY_INFO,
			"Stream Delay",
			STREAM_DELAY_INFO_MIN,
			STREAM_DELAY_INFO_MAX,
			1
		);
		obs_property_int_set_suffix(property, "ms");
		obs_property_set_enabled(property, false);

		property = obs_properties_add_int_slider(
			properties,
			PROP_CROP_PERCENTAGE,
			"Crop",
			CROP_PERCENTAGE_MIN,
			CROP_PERCENTAGE_MAX,
			1
		);
		obs_property_int_set_suffix(property, "%");

		property = obs_properties_add_list(
			properties,
			PROP_MOTION_MODEL,
			"Motion Model",
			OBS_COMBO_TYPE_LIST,
			OBS_COMBO_FORMAT_STRING
		);
		obs_property_list_add_string(property, "Dynamic", MOTION_MODEL_DYNAMIC);
		obs_property_list_add_string(property, "Affine", MOTION_MODEL_AFFINE);
		obs_property_list_add_string(property, "Homography", MOTION_MODEL_HOMOGRAPHY);

		property = obs_properties_add_list(
			properties,
			PROP_SUPPRESSION_MODE,
			"Suppression Mode",
			OBS_COMBO_TYPE_LIST,
			OBS_COMBO_FORMAT_STRING
		);
		obs_property_list_add_string(property, "Off", SUPPRESSION_MODE_OFF);
		obs_property_list_add_string(property, "Strict", SUPPRESSION_MODE_STRICT);
		obs_property_list_add_string(property, "Relaxed", SUPPRESSION_MODE_RELAXED);

		obs_properties_add_bool(
			properties,
			PROP_STAB_DISABLED,
			"Disable Stabilization"
		);

		obs_properties_add_bool(
			properties,
			PROP_TEST_MODE,
			"Test Mode"
		);

		return properties;
	}

//---------------------------------------------------------------------------------------------------------------------

	void VSFilter::LoadDefaults(obs_data_t* settings)
	{
		LVK_ASSERT(settings != nullptr);

		obs_data_set_default_int(settings, PROP_SMOOTHING_RADIUS, SMOOTHING_RADIUS_DEFAULT);
		obs_data_set_default_int(settings, PROP_CROP_PERCENTAGE, CROP_PERCENTAGE_DEFAULT);
		obs_data_set_default_string(settings, PROP_MOTION_MODEL, MOTION_MODEL_DEFAULT);
		obs_data_set_default_string(settings, PROP_SUPPRESSION_MODE, SUPPRESSION_MODE_DEFAULT);
		obs_data_set_default_bool(settings, PROP_STAB_DISABLED, STAB_DISABLED_DEFAULT);
		obs_data_set_default_bool(settings, PROP_TEST_MODE, TEST_MODE_DEFAULT);
	}

	//---------------------------------------------------------------------------------------------------------------------

	void VSFilter::configure(obs_data_t* settings)
	{
		LVK_ASSERT(settings != nullptr);

		// Update smoothing radius
		const uint32_t new_radius = round_even(obs_data_get_int(settings, PROP_SMOOTHING_RADIUS));
		if(m_SmoothingRadius != new_radius)
		{
			m_SmoothingRadius = new_radius;
			reset_buffers();
		}

		// Update motion model
		const std::string new_model = obs_data_get_string(settings, PROP_MOTION_MODEL);
		if(new_model == MOTION_MODEL_AFFINE)
			m_FrameTracker.set_model(MotionModel::AFFINE);
		else if(new_model == MOTION_MODEL_HOMOGRAPHY)
			m_FrameTracker.set_model(MotionModel::HOMOGRAPHY);
		else if(new_model == MOTION_MODEL_DYNAMIC)
			m_FrameTracker.set_model(MotionModel::DYNAMIC);

		// Update suppression mode
		const std::string new_mode = obs_data_get_string(settings, PROP_SUPPRESSION_MODE);
		if(new_mode == SUPPRESSION_MODE_OFF)
			m_SuppressionRange = SUPPRESSION_RANGE_OFF;
		else if(new_mode == SUPPRESSION_MODE_STRICT)
			m_SuppressionRange = SUPPRESSION_RANGE_STRICT;
		else if(new_mode == SUPPRESSION_MODE_RELAXED)
			m_SuppressionRange = SUPPRESSION_RANGE_RELAXED;

		// NOTE: If stabilisation is disabled, we need to restart the FrameTracker
		// so that is starts from scratch when its turned on again. Otherwise it
		// will try compare an old frame with a new one leading to bad output.
		m_Enabled = !obs_data_get_bool(settings, PROP_STAB_DISABLED);
		if(!m_Enabled)
			m_FrameTracker.restart();

		m_CropProportion = obs_data_get_int(settings, PROP_CROP_PERCENTAGE) / 100.0f;
		m_TestMode = obs_data_get_bool(settings, PROP_TEST_MODE);

		// Update stream delay indication for the user
		obs_video_info video_info;
		obs_get_video_info(&video_info);
		const float frame_ms = 1000.0 * video_info.fps_den / video_info.fps_num;

		const uint32_t stream_delay = obs_data_get_int(settings, PROP_STREAM_DELAY_INFO);
		const uint32_t new_stream_delay = frame_ms * m_FrameQueue.window_size();

		// NOTE: Need to update the property UI to push a stream delay update because
		// the UI element is disabled. But only if the delay has changed, otherwise
		// the sliders are interrupted and won't smoothly drag anymore.
		if(stream_delay != new_stream_delay)
		{
			obs_data_set_int(settings, PROP_STREAM_DELAY_INFO, new_stream_delay);
			obs_source_update_properties(m_Context);
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	VSFilter::VSFilter(obs_source_t* context)
		: VisionFilter(context),
		  m_Context(context),
		  m_Enabled(true),
		  m_TestMode(false),
		  m_SmoothingRadius(0),
		  m_CropProportion(0),
		  m_OutputSize(0, 0),
		  m_WarpFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_TrackingFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_FrameTracker(/* Use defaults */),
		  m_SuppressionRange(0.0),
		  m_SuppressionFactor(0.0)
	{
		LVK_ASSERT(context != nullptr);
	}

//---------------------------------------------------------------------------------------------------------------------

	void VSFilter::tick()
	{
		if(is_stabilisation_ready())
		{
			// NOTE: Next frame is the second oldest frame in the queue
			auto frame_size = m_FrameQueue[1].frame.size();

			m_CropRegion = crop(frame_size, m_CropProportion);
			m_OutputSize = frame_size;
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void VSFilter::hybrid_render(gs_texture_t* frame)
	{
		if (frame == nullptr)
		{
			// As Video Filter
			if (m_TestMode)
				obs_source_skip_video_filter(m_Context);
			else if (!FSREffect::Render(m_Context, m_OutputSize, m_CropRegion))
				obs_source_skip_video_filter(m_Context);
		}
		else
		{
			// As Effects Filter
			if(m_TestMode)
				DefaultEffect::Render(frame);
			else if (!FSREffect::Render(frame, m_OutputSize, m_CropRegion))
				obs_source_skip_video_filter(m_Context);
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void VSFilter::filter(FrameBuffer& buffer)
	{
		uint64_t start_time = os_gettime_ns();

		if(is_queue_outdated(buffer))
		{
			reset_buffers();
			m_FrameTracker.restart();
			log::warn("\'%s\' frame queue is outdated, restarting...", obs_source_get_name(m_Context));
		}

		Homography tracked_motion;
		if (m_Enabled)
		{
			cv::extractChannel(buffer.frame, m_TrackingFrame, 0);
			tracked_motion = m_FrameTracker.track(m_TrackingFrame);
		}

		auto& frame_vector = m_Trajectory.advance();
		frame_vector.velocity = suppress(tracked_motion);
		frame_vector.displacement = m_Trajectory.previous().displacement + frame_vector.velocity;
		
		if(m_Enabled && m_TestMode)
		{
			start_time += draw_debug_frame(
				buffer.frame,
				m_FrameTracker.stability(),
				m_FrameTracker.tracking_points()
			);
		}

		m_FrameQueue.push(std::move(buffer));

		if(is_stabilisation_ready())
		{
			FrameBuffer& output = m_FrameQueue.oldest();

			if(m_Enabled)
			{
				const auto& [displacement, velocity] = m_Trajectory.centre();

				const auto trajectory_correction = m_Trajectory.convolve_at(m_Filter, m_Trajectory.centre_index()).displacement - displacement;
				const auto stabilised_velocity = clamp_velocity(output.frame, velocity + trajectory_correction);

				stabilised_velocity.warp(output.frame, m_WarpFrame);
				m_WarpFrame.copyTo(output.frame);
			}

			if(m_TestMode)
			{
				cv::ocl::finish();
				draw_debug_hud(
					output.frame,
					os_gettime_ns() - start_time
				);
			}

			buffer = std::move(output);
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	Homography VSFilter::clamp_velocity(const cv::UMat& frame, const Homography& velocity)
	{
		// Clamp the velocity to keep the crop enclosed within the warped frame
		// ensuring that no 'green bars' are present. This is done by iteratively
		// reducing the velocity by lerping it back to identity in small steps.

		constexpr double max_t = 1.0;
		constexpr int max_iterations = 100;
		constexpr double step = max_t/max_iterations;
		const auto identity = Homography::Identity();

		double t = step;
		Homography reduced_velocity = velocity;
		BoundingQuad frame_bounds(frame.size(), reduced_velocity);

		while(t <= max_t && !frame_bounds.encloses(m_CropRegion))
		{
			reduced_velocity = lerp(velocity, identity, t);
			frame_bounds.transform(reduced_velocity);
			t += step;
		}

		return reduced_velocity;
	}

//---------------------------------------------------------------------------------------------------------------------

	uint64_t VSFilter::draw_debug_frame(
		cv::UMat& frame,
		const float tracking_stability,
		const std::vector<cv::Point2f>& trackers
	)
	{
		cv::ocl::finish();
		const uint64_t start_time = os_gettime_ns();

		draw::plot_markers(
			frame,
			m_FrameTracker.tracking_points(),
			lerp(draw::YUV_GREEN, draw::YUV_RED, m_SuppressionFactor),
			cv::MarkerTypes::MARKER_CROSS,
			8,
			2
		);

		cv::ocl::finish();
		return os_gettime_ns() - start_time;
	}

//---------------------------------------------------------------------------------------------------------------------

	void VSFilter::draw_debug_hud(cv::UMat& frame, const uint64_t frame_time_ns)
	{
		draw::rect(frame, m_CropRegion, draw::YUV_MAGENTA);

		const double frame_time_ms = frame_time_ns * 1.0e-6;

		//TODO: switch to C++20 fmt as soon as GCC supports it.
		std::stringstream time_text;
		time_text << std::fixed << std::setprecision(2);
		time_text << frame_time_ms << "ms";

		draw::text(
			frame,
			time_text.str(),
			m_CropRegion.tl() + cv::Point(5, 40),
			frame_time_ms < TIMING_THRESHOLD_MS ? draw::YUV_GREEN : draw::YUV_RED
		);

	}

//---------------------------------------------------------------------------------------------------------------------

	void VSFilter::reset_buffers()
	{
		LVK_ASSERT(m_SmoothingRadius >= SMOOTHING_RADIUS_MIN);
		LVK_ASSERT(m_SmoothingRadius % 2 == 0);

		// NOTE: Must release all OBS frames before clearing
		// the frame buffer queue to avoid leaking memory.
		m_FrameQueue.clear();
		m_Trajectory.clear();

		// NOTE: Trajectory uses a full window for stabilising the centre element,
		// so the frame queue needs to supply delayed frames up to the centre.
		// Tracking is performed on the newest frame but the tracked velocity
		// has to be associated with the previous frame, so we add another frame
		// to the queue to introduce an offset.

		const uint32_t queue_size = m_SmoothingRadius + 2;
		const uint32_t window_size = 2 * m_SmoothingRadius + 1;

		if(window_size != m_Trajectory.window_size() || queue_size != m_FrameQueue.window_size())
		{
			m_FrameQueue.resize(queue_size);
			m_Trajectory.resize(window_size);
			m_Filter.resize(window_size);

			// NOTE: A low pass Gaussian filter is used because it has both decent time domain
			// and frequency domain performance. Unlike an average or windowed sinc filter.
			// As a rule of thumb, sigma is chosen to fit 99.7% of the distribution in the window.
			const auto gaussian_kernel = cv::getGaussianKernel(window_size, window_size/6.0);

			m_Filter.clear();
			for(uint32_t i = 0; i < window_size; i++)
				m_Filter.push(gaussian_kernel.at<double>(i));
		}

		// Fill the trajectory to bring the buffers into synchronisation.
		m_Trajectory.advance(Homography::Identity());
		while(m_Trajectory.elements() < m_SmoothingRadius - 1)
			m_Trajectory.advance(m_Trajectory.newest() + Homography::Identity());
	}

//---------------------------------------------------------------------------------------------------------------------

	bool VSFilter::is_queue_outdated(const FrameBuffer& new_frame) const
	{
		// If the new frame is over a second away from the previously newest 
		// frame, then we consider the trajectory and frame data to be outdated.

		// NOTE: If the new frame is actually older than the previously 
		// newest frame, the unsigned timestamps will overflow and the queue 
		// always will be flagged as outdated. This is the desired behaviour.
		return !m_FrameQueue.empty()
			&& new_frame.timestamp - m_FrameQueue.newest().timestamp > 1e9;
	}

//---------------------------------------------------------------------------------------------------------------------

	Homography VSFilter::suppress(Homography& motion)
	{
		const float scene_stability = m_FrameTracker.stability();
		const float suppression_threshold = m_SuppressionRange.y;
		const float suppression_limit = m_SuppressionRange.x;

		float suppression_target = 0.0f;
		if (between(scene_stability, suppression_limit, suppression_threshold))
		{
			const float length = suppression_threshold - suppression_limit;
			suppression_target = 1.0f - ((scene_stability - suppression_limit) / length);
		}
		else if(!m_Enabled || scene_stability < suppression_limit)
			suppression_target = 1.0f;

		m_SuppressionFactor = step(
			m_SuppressionFactor,
			suppression_target,
			delta_time() * SUPPRESSION_SMOOTHING_STEP
		);

		return (1.0f - m_SuppressionFactor) * motion + m_SuppressionFactor * Homography::Identity();
	}

//---------------------------------------------------------------------------------------------------------------------

	bool VSFilter::is_stabilisation_ready() const
	{
		LVK_ASSERT(m_Trajectory.full() == m_FrameQueue.full());

		return m_Trajectory.full()
			&& m_FrameQueue.full();
	}

//---------------------------------------------------------------------------------------------------------------------

	uint32_t VSFilter::width() const
	{
		return m_OutputSize.width;
	}

//---------------------------------------------------------------------------------------------------------------------

	uint32_t VSFilter::height() const
	{
		return m_OutputSize.height;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool VSFilter::validate() const
	{
		return m_Context != nullptr
			&& FSREffect::Validate();
	}

//---------------------------------------------------------------------------------------------------------------------

	FrameVector::FrameVector(const Homography& displacement, const Homography& velocity)
		: displacement(displacement),
		  velocity(velocity)
	{}

//---------------------------------------------------------------------------------------------------------------------

	FrameVector FrameVector::operator+(const FrameVector& other) const
	{
		return FrameVector(displacement + other.displacement, velocity + other.velocity);
	}

//---------------------------------------------------------------------------------------------------------------------

	FrameVector FrameVector::operator-(const FrameVector& other) const
	{
		return FrameVector(displacement - other.displacement, velocity - other.velocity);
	}


//---------------------------------------------------------------------------------------------------------------------

	FrameVector FrameVector::operator*(const double scaling) const
	{
		return FrameVector(displacement * scaling, velocity * scaling);
	}

//---------------------------------------------------------------------------------------------------------------------

	FrameVector FrameVector::operator/(const double scaling) const
	{
		return FrameVector(displacement / scaling, velocity / scaling);
	}

//---------------------------------------------------------------------------------------------------------------------

}

