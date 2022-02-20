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

#include <opencv2/core/ocl.hpp>
#include <sstream>

#include "../../Interop/FrameIngest.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	static constexpr auto PROP_SMOOTHING_RADIUS = "SMOOTH_RADIUS";
	static constexpr auto SMOOTHING_RADIUS_DEFAULT = 10;
	static constexpr auto SMOOTHING_RADIUS_MIN = 2;
	static constexpr auto SMOOTHING_RADIUS_MAX = 16;

	static constexpr auto PROP_STREAM_DELAY_INFO = "STREAM_DELAY_INFO";
	static constexpr auto STREAM_DELAY_INFO_MIN = 0;
	static constexpr auto STREAM_DELAY_INFO_MAX = 100 * SMOOTHING_RADIUS_MAX;

	static constexpr auto PROP_CROP_PERCENTAGE = "CROP_PERCENTAGE";
	static constexpr auto CROP_PERCENTAGE_DEFAULT = 8;
	static constexpr auto CROP_PERCENTAGE_MIN = 1;
	static constexpr auto CROP_PERCENTAGE_MAX = 25;

	static constexpr auto PROP_MOTION_MODEL = "MOTION_MODEL";
	static constexpr auto MOTION_MODEL_AFFINE = "AFFINE";
	static constexpr auto MOTION_MODEL_HOMOGRAPHY = "HOMOGRAPHY";
	static constexpr auto MOTION_MODEL_DEFAULT = MOTION_MODEL_HOMOGRAPHY;

	static constexpr auto PROP_STAB_DISABLED = "STAB_DISABLED";
	static constexpr auto STAB_DISABLED_DEFAULT = false;

	static constexpr auto PROP_TEST_MODE = "TEST_MODE";
	static constexpr auto TEST_MODE_DEFAULT = false;

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
		obs_property_list_add_string(property, "Affine", MOTION_MODEL_AFFINE);
		obs_property_list_add_string(property, "Homography", MOTION_MODEL_HOMOGRAPHY);

		obs_properties_add_bool(
			properties,
			PROP_STAB_DISABLED,
			"Disable Stabilisation"
		);

		obs_properties_add_bool(
			properties,
			PROP_TEST_MODE,
			"Test Mode"
		);

		return properties;
	}

//---------------------------------------------------------------------------------------------------------------------

	void VSFilter::LoadDefault(obs_data_t* settings)
	{
		LVK_ASSERT(settings != nullptr);

		obs_data_set_default_int(settings, PROP_SMOOTHING_RADIUS, SMOOTHING_RADIUS_DEFAULT);
		obs_data_set_default_int(settings, PROP_CROP_PERCENTAGE, CROP_PERCENTAGE_DEFAULT);
		obs_data_set_default_string(settings, PROP_MOTION_MODEL, MOTION_MODEL_DEFAULT);
		obs_data_set_default_bool(settings, PROP_STAB_DISABLED, STAB_DISABLED_DEFAULT);
		obs_data_set_default_bool(settings, PROP_TEST_MODE, TEST_MODE_DEFAULT);
	}

//---------------------------------------------------------------------------------------------------------------------

	VSFilter* VSFilter::Create(obs_source_t* context, obs_data_t* settings)
	{
		LVK_ASSERT(context != nullptr && settings != nullptr);

		auto filter = new VSFilter(context);

		if(!filter->validate())
		{
			delete filter;
			return nullptr;
		}

		cv::ocl::setUseOpenCL(true);

		filter->configure(settings);

		return filter;
	}

//---------------------------------------------------------------------------------------------------------------------

	VSFilter::VSFilter(obs_source_t* context)
		: m_Context(context),
		  m_Shader(nullptr),
		  m_CropParam(nullptr),
		  m_Enabled(true),
		  m_TestMode(false),
		  m_SmoothingRadius(0),
		  m_CropProportion(0),
		  m_OutputSize(0, 0),
		  m_WarpFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_TrackingFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_FrameTracker(/* Use defaults */)
	{
		LVK_ASSERT(context != nullptr);

		char* shader_path = obs_module_file("effects/vs.effect");
		if(shader_path != nullptr)
		{
			obs_enter_graphics();

			m_Shader = gs_effect_create_from_file(shader_path, nullptr);
			bfree(shader_path);

			if(m_Shader)
				m_CropParam = gs_effect_get_param_by_name(m_Shader, "crop_proportion");

			obs_leave_graphics();
		}



	}

//---------------------------------------------------------------------------------------------------------------------

	VSFilter::~VSFilter()
	{
		release_frame_queue();
	}

//---------------------------------------------------------------------------------------------------------------------

	void VSFilter::configure(obs_data_t* settings)
	{
		LVK_ASSERT(settings != nullptr);

		const uint32_t new_radius = round_even(obs_data_get_int(settings, PROP_SMOOTHING_RADIUS));
		if(m_SmoothingRadius != new_radius)
		{
			m_SmoothingRadius = new_radius;
			reset_buffers();
		}

		const std::string new_model = obs_data_get_string(settings, PROP_MOTION_MODEL);
		if(new_model == MOTION_MODEL_AFFINE)
			m_FrameTracker.set_model(MotionModel::AFFINE);
		else if(new_model == MOTION_MODEL_HOMOGRAPHY)
			m_FrameTracker.set_model(MotionModel::HOMOGRAPHY);

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

	void VSFilter::tick()
	{
		if(stabilisation_ready())
		{
			auto frame_size = m_FrameQueue.oldest().frame.size();

			m_CropRegion = crop(frame_size, m_CropProportion);

			if(m_TestMode)
				m_OutputSize = frame_size;
			else
				m_OutputSize = m_CropRegion.size();
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void VSFilter::render() const
	{
		if(obs_source_process_filter_begin(m_Context, GS_RGBA, OBS_NO_DIRECT_RENDERING))
		{
			if(m_TestMode)
				gs_effect_set_float(m_CropParam, 0.0f);
			else
				gs_effect_set_float(m_CropParam, m_CropProportion);

			obs_source_process_filter_end(m_Context, m_Shader, m_OutputSize.width, m_OutputSize.height);
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	obs_source_frame* VSFilter::process(obs_source_frame* obs_frame)
	{
		LVK_ASSERT(obs_frame != nullptr);

		uint64_t start_time = os_gettime_ns();

		if(is_queue_outdated(obs_frame))
			reset();

		auto& buffer = m_FrameQueue.advance();
		buffer.frame << obs_frame;
		buffer.output = obs_frame;

		// Extract Y channel from YUV frame to use for tracking.
		cv::extractChannel(buffer.frame, m_TrackingFrame, 0);

		auto& motion = m_Trajectory.advance();
		motion.velocity = m_Enabled ? m_FrameTracker.track(m_TrackingFrame) : Homography::Identity();
		motion.displacement = m_Trajectory.previous().displacement + motion.velocity;

		if(m_TestMode && m_Enabled)
			start_time += draw_debug_frame(buffer.frame, m_FrameTracker.tracking_points());

		if(stabilisation_ready())
		{
			// NOTE: Must forcibly remove the OBS frame to avoid accidentally
			// releasing it later and causing a hard to find double free issue :)
			auto [frame, output] = m_FrameQueue.oldest();
			m_FrameQueue.oldest().output = nullptr;

			if(m_Enabled)
			{
				const auto& [displacement, velocity] = m_Trajectory.centre();

				const auto trajectory_correction = m_Trajectory.convolve(m_Filter).displacement - displacement;
				const auto stabilised_velocity = clamp_velocity(frame, velocity + trajectory_correction);

				stabilised_velocity.warp(frame, m_WarpFrame);

				m_WarpFrame >> output;
			}

			if(m_TestMode)
			{
				draw_debug_hud(
					m_Enabled ? m_WarpFrame : frame,
					os_gettime_ns() - start_time
				) >> output;
			}

			return output;
		}
		return nullptr;
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
		auto reduced_velocity = velocity;
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

	uint64_t VSFilter::draw_debug_frame(cv::UMat& frame, const std::vector<cv::Point2f>& trackers)
	{
		const uint64_t start_time = os_gettime_ns();

		const cv::Scalar green_yuv(149, 43, 21);

		plot_markers(
			frame,
			m_FrameTracker.tracking_points(),
			green_yuv,
			cv::MarkerTypes::MARKER_CROSS,
			8,
			2
		);

		return os_gettime_ns() - start_time;
	}

//---------------------------------------------------------------------------------------------------------------------

	cv::UMat VSFilter::draw_debug_hud(cv::UMat& frame, const uint64_t frame_time_ns)
	{
		const cv::Scalar magenta_yuv(105, 212, 234);
		const cv::Scalar green_yuv(149, 43, 21);
		const cv::Scalar red_yuv(76, 84, 255);

		cv::rectangle(frame, m_CropRegion, magenta_yuv, 2);

		const double bad_time_threshold_ms = 8.0;
		const double frame_time_ms = frame_time_ns * 1.0e-6;

		//TODO: switch to C++20 fmt as soon as GCC supports it.
		std::stringstream time_text;
		time_text << std::fixed << std::setprecision(2);
		time_text << frame_time_ms << "ms";

		cv::putText(
			frame,
			time_text.str(),
			m_CropRegion.tl() + cv::Point(5, 40),
			cv::FONT_HERSHEY_DUPLEX,
			1.5,
			frame_time_ms < bad_time_threshold_ms ? green_yuv : red_yuv,
			2
		);

		return frame;
	}

//---------------------------------------------------------------------------------------------------------------------

	void VSFilter::reset()
	{
		reset_buffers();
		m_FrameTracker.restart();

		// Release GPU buffers to save memory
		m_TrackingFrame.release();
		m_WarpFrame.release();
	}

//---------------------------------------------------------------------------------------------------------------------

	void VSFilter::reset_buffers()
	{
		LVK_ASSERT(m_SmoothingRadius >= SMOOTHING_RADIUS_MIN && m_SmoothingRadius % 2 == 0);

		// NOTE: Must release all OBS frames before clearing
		// the frame buffer queue to avoid leaking memory.
		release_frame_queue();
		m_Trajectory.clear();

		// NOTE: Trajectory uses a full window for stabilising the centre element,
		// so the frame queue needs to supply delayed frames up to the centre.
		// Tracking is performed on the newest frame but the tracked velocity
		// has to be associated with the previous frame, so we add another frame
		// to the queue to introduce an offset.

		const uint32_t queue_size = m_SmoothingRadius + 2;
		const uint32_t window_size = 2 * m_SmoothingRadius + 1;

		if(window_size != m_Trajectory.window_size())
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

	void VSFilter::release_frame_queue()
	{
		// Release all frames to save GPU memory and prevent memory leaks
		obs_source_t* parent = obs_filter_get_parent(m_Context);
		for(uint32_t i = 0; i < m_FrameQueue.elements(); i++)
		{
			auto& [frame, obs_frame] = m_FrameQueue[i];

			frame.release();
			obs_source_release_frame(parent, obs_frame);
		}
		m_FrameQueue.clear();
	}

//---------------------------------------------------------------------------------------------------------------------

	bool VSFilter::is_queue_outdated(const obs_source_frame* new_frame) const
	{
		// If the frame is over a second away from the last frame
		// then we consider the trajectory and frame data to be outdated.
		return !m_FrameQueue.empty()
			&& new_frame->timestamp - m_FrameQueue.newest().output->timestamp > 1e9;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool VSFilter::stabilisation_ready() const
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
			&& m_Shader != nullptr
			&& m_CropParam != nullptr;
	}

//---------------------------------------------------------------------------------------------------------------------

	VSFilter::FrameBuffer::FrameBuffer()
		: frame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  output(nullptr)
	{}

//---------------------------------------------------------------------------------------------------------------------

	VSFilter::FrameVector::FrameVector(const Homography& displacement, const Homography& velocity)
		: displacement(displacement),
		  velocity(velocity)
	{}

//---------------------------------------------------------------------------------------------------------------------

	VSFilter::FrameVector VSFilter::FrameVector::operator+(const VSFilter::FrameVector& other) const
	{
		return FrameVector(displacement + other.displacement, velocity + other.velocity);
	}

//---------------------------------------------------------------------------------------------------------------------

	VSFilter::FrameVector VSFilter::FrameVector::operator-(const VSFilter::FrameVector& other) const
	{
		return FrameVector(displacement - other.displacement, velocity - other.velocity);
	}


//---------------------------------------------------------------------------------------------------------------------

	VSFilter::FrameVector VSFilter::FrameVector::operator*(const double scaling) const
	{
		return FrameVector(displacement * scaling, velocity * scaling);
	}

//---------------------------------------------------------------------------------------------------------------------

	VSFilter::FrameVector VSFilter::FrameVector::operator/(const double scaling) const
	{
		return FrameVector(displacement / scaling, velocity / scaling);
	}

//---------------------------------------------------------------------------------------------------------------------

}

