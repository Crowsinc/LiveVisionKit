#include "VSFilter.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>

#include <util/platform.h>
#include <util/threading.h>

namespace lvk
{

	//===================================================================================
	//		CONSTANT PROPERTIES/SETTINGS
	//===================================================================================

	static const FrameTracker::Properties TRACKING_PROPERTIES = {/* Use defaults */};

	static constexpr auto PROP_SMOOTHING_RADIUS = "SMOOTH_RADIUS";
	static constexpr auto SMOOTHING_RADIUS_DEFAULT = 14;

	static constexpr auto PROP_CROP_PERCENTAGE = "CROP_PERCENTAGE";
	static constexpr auto CROP_PERCENTAGE_DEFAULT = 5;

	static constexpr auto PROP_AUTO_UPSCALE = "AUTO_UPSCALE";
	static constexpr auto AUTO_UPSCALE = true;

	static constexpr auto PROP_TEST_MODE = "TEST_MODE";
	static constexpr auto TEST_MODE_DEFAULT = false;

	//===================================================================================
	//		FILTER IMPLEMENTATION
	//===================================================================================

	obs_properties_t* VSFilter::Properties()
	{
		obs_properties_t* properties = obs_properties_create();

		// Slider for window radius.
		// Capped at 30 to avoid people using insanely high delays
		obs_properties_add_int_slider(
				properties,
				PROP_SMOOTHING_RADIUS,
				"Smoothing Radius (Frame Delay)",
				2,
				30,
				2
		);

		// Slider for total proportion of allowable crop along each dimension.
		// The amount of pixels to crop at each edge is: dimension * (crop/2/100)
		obs_properties_add_int_slider(
				properties,
				PROP_CROP_PERCENTAGE,
				"Crop Percentage",
				1,
				25,
				1
		);

		// Toggle for custom upscaler, setting whether the output is automatically scaled or not.
		obs_properties_add_bool(
				properties,
				PROP_AUTO_UPSCALE,
				"Perform Upscaling"
		);

		// Toggle for test mode, used to help configure settings.
		obs_properties_add_bool(
				properties,
				PROP_TEST_MODE,
				"Test Mode"
		);


		return properties;
	}

	//-------------------------------------------------------------------------------------

	void VSFilter::LoadDefault(obs_data_t* settings)
	{
		obs_data_set_default_int(settings, PROP_SMOOTHING_RADIUS,  SMOOTHING_RADIUS_DEFAULT);
		obs_data_set_default_int(settings, PROP_CROP_PERCENTAGE,  CROP_PERCENTAGE_DEFAULT);
		obs_data_set_default_bool(settings, PROP_TEST_MODE, TEST_MODE_DEFAULT);
		obs_data_set_default_bool(settings, PROP_AUTO_UPSCALE, AUTO_UPSCALE);
	}

	//-------------------------------------------------------------------------------------

	VSFilter* VSFilter::Create(obs_source_t* context)
	{
		auto filter = new VSFilter(context);

		if(!filter->validate())
		{
			delete filter;
			return nullptr;
		}

		// Configure OpenCV to use OpenCL
		cv::ocl::setUseOpenCL(true);

		return filter;
	}

	//-------------------------------------------------------------------------------------

	VSFilter::VSFilter(obs_source_t* context)
		: m_Context(context),
		  m_TestMode(false),
		  m_AutoUpscale(false),
		  m_EdgeCropProportion(0),
		  m_SmoothingRadius(0),
		  m_WarpFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_StabilisedFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_TrackingFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_FrameTracker(TRACKING_PROPERTIES)
	{
		m_Shader = obs_get_base_effect(OBS_EFFECT_DEFAULT);
	}

	//-------------------------------------------------------------------------------------

	VSFilter::~VSFilter()
	{
		reset();
	}

	//-------------------------------------------------------------------------------------

	void VSFilter::render() const
	{
		if(!obs_source_process_filter_begin(m_Context, GS_RGBA, OBS_NO_DIRECT_RENDERING))
			return;

		obs_source_process_filter_end(m_Context, m_Shader, m_OutputSize.width, m_OutputSize.height);
	}

	//-------------------------------------------------------------------------------------

	void VSFilter::configure(obs_data_t* settings)
	{
		const uint32_t new_smoothing_radius = obs_data_get_int(settings, PROP_SMOOTHING_RADIUS);
		if(m_SmoothingRadius != new_smoothing_radius)
		{
			// Only update smoothing radius if it has changed
			// because it requires resetting all the buffers
			m_SmoothingRadius = new_smoothing_radius;
			prepare_buffers(new_smoothing_radius);
		}

		// Crop proportion is shared by opposite edges, so we also divide by 2
		m_EdgeCropProportion = obs_data_get_int(settings, PROP_CROP_PERCENTAGE) / 200.0f;

		m_TestMode = obs_data_get_bool(settings, PROP_TEST_MODE);
		m_AutoUpscale  = obs_data_get_bool(settings, PROP_AUTO_UPSCALE);
	}

	//-------------------------------------------------------------------------------------

	void VSFilter::tick()
	{
		if(!m_StabilisationQueue.empty())
		{
			const auto next_frame = m_StabilisationQueue.oldest();

			// Original region of the frame
			m_FrameRegion.width = next_frame.cols;
			m_FrameRegion.height = next_frame.rows;

			const uint32_t horz_edge_crop = m_FrameRegion.width * m_EdgeCropProportion;
			const uint32_t vert_edge_crop = m_FrameRegion.height * m_EdgeCropProportion;

			// Resulting region after crop
			m_CropRegion.x = horz_edge_crop;
			m_CropRegion.y = vert_edge_crop;
			m_CropRegion.width = m_FrameRegion.width - 2 * horz_edge_crop;
			m_CropRegion.height = m_FrameRegion.height - 2 * vert_edge_crop;

			if(m_TestMode || m_AutoUpscale)
				m_OutputSize = m_FrameRegion.size();
			else
				m_OutputSize = m_CropRegion.size();
		}
	}

	//-------------------------------------------------------------------------------------

	obs_source_frame* VSFilter::process(obs_source_frame* next_frame)
	{
		const uint64_t t1 = os_gettime_ns();

		// Ingest next frame into the stabilisation queue
		m_StabilisationQueue.advance(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY) << next_frame;
		m_OBSFrameQueue.push(next_frame);

		const uint64_t t2 = os_gettime_ns();

		// Frame tracking operates only on Y plane
		cv::extractChannel(m_StabilisationQueue.newest(), m_TrackingFrame, 0);

		// Track the frame, recording the estimated motion & path data
		m_MotionQueue.push(m_FrameTracker.track(m_TrackingFrame));
		m_PathWindow.push(m_FrameTracker.cumulative());

		// Perform stabilisation
		if(stabilisation_ready())
		{
			auto input_frame = m_StabilisationQueue.oldest();
			auto output_obs_frame = m_OBSFrameQueue.oldest();

			// Apply the motion filter on the path to get the desired smooth path for the frame
			// being processed. We then correct the estimated motion to end on the smooth path.
			const Transform warp_correction = m_PathWindow.convolve(m_PathFilter) - m_PathWindow.centre();
			const auto smooth_warp = m_MotionQueue.oldest() + warp_correction;

			auto mot = m_PathWindow.centre();
			blog(LOG_INFO, "MOTION ESTIMATE: (%f, %f, %f, %f)", mot.translation.x, mot.translation.y, mot.rotation, mot.scale);

//	TODO:		const auto safe_warp = fit_to_crop(smooth_motion, input_frame, crop_region);

			cv::warpAffine(input_frame, m_WarpFrame, smooth_warp.as_matrix(), input_frame.size());

			const uint64_t t3 = os_gettime_ns();

			if(!m_TestMode)
			{
				// Apply the crop, resizing the frame to fit back into the input OBS frame
				cv::resize(m_WarpFrame(m_CropRegion), m_StabilisedFrame, input_frame.size(), 0, 0, cv::INTER_CUBIC);
				m_StabilisedFrame >> output_obs_frame;
			}
			else draw_test_information(m_WarpFrame, t2 - t1, t3 - t2) >> output_obs_frame;

			return output_obs_frame;
		}

		//TODO: assert that none of the windows are full, because they should all become full at the same time

		return nullptr;
	}

	//-------------------------------------------------------------------------------------

	Transform VSFilter::fit_to_crop(const Transform& transform)
	{
		// Reduce the magnitude of the transform until the crop region is
		// fully enclosed within the warped frame. We reduce the magnitude
		// by iteratively lerping the transform back to identity in small steps.

		constexpr double max_t = 1.0;
		constexpr int max_iterations = 100;
		constexpr double step = max_t/max_iterations;
		const auto identity = Transform::Identity();

		double t = 0.0;
		auto reduced_transform = transform;
		while(t <= max_t && !encloses(m_FrameRegion, reduced_transform, m_CropRegion))
		{
			reduced_transform = lerp(transform, identity, t);
			t += step;
		}

		return reduced_transform;
	}

	//-------------------------------------------------------------------------------------

	cv::UMat VSFilter::draw_test_information(cv::UMat& frame, const uint64_t ingest_time_ns, const uint64_t warp_time_ns)
	{
		const double ingest_time_ms = ingest_time_ns * 1.0e-6;
		const double warp_time_ms = warp_time_ns * 1.0e-6;
		const double est_frame_time_ms = 2.0 * ingest_time_ms + warp_time_ms;

		const cv::Scalar yuv_magenta(105, 212, 234);

		const std::string ingest_text = "Ingest Time: " + std::to_string(ingest_time_ms) + "ms";
		const std::string warp_text = "Warp Time: " + std::to_string(warp_time_ms) + "ms";
		const std::string frame_text = "Frame Time: " + std::to_string(est_frame_time_ms) + "ms";

		cv::putText(frame, ingest_text, cv::Point(20, 70), cv::FONT_HERSHEY_PLAIN, 2, yuv_magenta, 2);
		cv::putText(frame, warp_text, cv::Point(20, 140), cv::FONT_HERSHEY_PLAIN, 2, yuv_magenta, 2);
		cv::putText(frame, frame_text, cv::Point(20, 210), cv::FONT_HERSHEY_PLAIN, 2, yuv_magenta, 2);

		cv::rectangle(frame, m_CropRegion, yuv_magenta, 2);

		return frame;
	}

	//-------------------------------------------------------------------------------------

	void VSFilter::prepare_buffers(const uint32_t filter_radius)
	{
		//TODO: assert even numbers

		// NOTE: Stabalisation is achieved by applying a windowed low pass filter
		// to the frame/camera's path to remove high frequency 'shaking'. Effective
		// filtering requires a full sized window which takes into account both past
		// and future frames, obtained by delaying the stream. Delay is introduced
		// via half-sized sliding buffers. Such that the oldest element corresponds
		// with the centre element in the full sized path buffer.

		const uint32_t half_window_size = filter_radius + 1;
		const uint32_t full_window_size = 2 * filter_radius + 1;

		m_MotionQueue.resize(half_window_size);
		m_OBSFrameQueue.resize(half_window_size);
		m_StabilisationQueue.resize(half_window_size);

		m_PathWindow.resize(full_window_size);
		m_PathFilter.resize(full_window_size);

		// We use a low pass Gaussian filter because it has both decent time domain
		// and frequency domain performance. Unlike an average or windowed sinc filter.
		// As a rule of thumb, sigma is chosen to fit 99.7% of the distribution in the window.
		const auto gaussian_pdf = cv::getGaussianKernel(full_window_size, full_window_size/6.0);
		for(uint32_t i = 0; i < full_window_size; i++)
			m_PathFilter.push(gaussian_pdf.at<double>(i));

		// Enforces synchronisation.
		reset_buffers();
	}

	//-------------------------------------------------------------------------------------

	void VSFilter::reset()
	{
		reset_buffers();
		m_FrameTracker.reset();
	}

	//-------------------------------------------------------------------------------------

	void VSFilter::reset_buffers()
	{
		// Need to release all the frames to prevent memory leaks
		for(uint32_t i = 0; i < m_OBSFrameQueue.elements(); i++)
			obs_source_release_frame(m_Context, m_OBSFrameQueue[i]);

		m_StabilisationQueue.clear();
		m_OBSFrameQueue.clear();
		m_MotionQueue.clear();
		m_PathWindow.clear();

		// Half fill the full sized path window to make the next element
		// push to its centre, synchronising it with the half sized buffers.
		for(uint32_t i = 0; i < m_PathWindow.centre_index(); i++)
			m_PathWindow.push(Transform::Identity());
	}

	//-------------------------------------------------------------------------------------

	bool VSFilter::stabilisation_ready() const
	{
		// Stabilisation is ready when all the buffers are filled, meaning the
		// required delay has been achieved for proper filtering of the camera path.
		return m_PathWindow.full()
			&& m_OBSFrameQueue.full()
			&& m_MotionQueue.full()
			&& m_StabilisationQueue.full();
	}

	//-------------------------------------------------------------------------------------

	uint32_t VSFilter::width() const
	{
		return m_OutputSize.width;
	}

	//-------------------------------------------------------------------------------------

	uint32_t VSFilter::height() const
	{
		return m_OutputSize.height;
	}

	//-------------------------------------------------------------------------------------

	bool VSFilter::validate() const
	{
		// NOTE: Must run through OpenCL for performance reasons.
		return cv::ocl::haveOpenCL()
			&& m_Shader != nullptr;
	}

	//-------------------------------------------------------------------------------------

}
