#include "VSFilter.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>

namespace lvk
{

	//===================================================================================
	//		CONSTANT PROPERTIES/SETTINGS
	//===================================================================================

	static const FrameTracker::Properties TRACKING_PROPERTIES = {/* Use defaults */};

	static constexpr auto PROP_SMOOTHING_RADIUS = "SMOOTH_RADIUS";
	static constexpr auto SMOOTHING_RADIUS_DEFAULT = 40;

	static constexpr auto PROP_CROP_PERCENTAGE = "CROP_PERCENTAGE";
	static constexpr auto CROP_PERCENTAGE_DEFAULT = 5;

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

		// Toggle for test mode, used to help configure settings
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

		return filter;
	}

	//-------------------------------------------------------------------------------------

	VSFilter::VSFilter(obs_source_t* context)
		: m_Context(context),
		  m_TestMode(false),
		  m_FrameTracker(TRACKING_PROPERTIES),
		  m_WarpFrame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY)
	{
		// Everything should initialize properly on the first call to configure
	}

	//-------------------------------------------------------------------------------------

	void VSFilter::configure(obs_data_t* settings)
	{

	}

	//-------------------------------------------------------------------------------------

	void VSFilter::update()
	{
		//TODO: reset history if frame size has changed
	}

	//-------------------------------------------------------------------------------------

	obs_source_frame* VSFilter::process(obs_source_frame* next_frame)
	{
		// Ingest next frame into the stabilisation queue
		m_StabilisationQueue.advance(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY) << next_frame;
		m_OBSFrameQueue.push(next_frame);

		// Track the frame, recording the estimated motion & path data
		m_MotionQueue.push(m_FrameTracker.track(m_StabilisationQueue.newest()));
		m_PathWindow.push(m_FrameTracker.cumulative());

		// Once the filter delay is achieved, stabilise the oldest frame in the queue
		if(m_PathWindow.full() && m_MotionQueue.full() && m_OBSFrameQueue.full() && m_StabilisationQueue.full())
		{
			// Apply the motion filter on the path to get the desired smooth path for the frame
			// being processed. We then correct the estimated motion to end on the smooth path.
			const auto motion_correction = m_PathWindow.convolve(m_PathFilter) - m_PathWindow.centre();
			const auto smooth_motion = clamp_to_frame(m_MotionQueue.oldest() + motion_correction);

			auto& input_frame = m_StabilisationQueue.oldest();
			cv::warpAffine(input_frame, m_WarpFrame, smooth_motion.as_matrix(), input_frame.size());

			// Finish the stabilisation by cropping the frame and downloading it into the OBS frame
			// Unless we are in test mode, where we draw test information instead of cropping.
			auto& output_frame = m_OBSFrameQueue.oldest();
			if(m_TestMode)
				draw_test_information(m_WarpFrame) >> output_frame;
			else
				input_frame(m_CropRegion) >> output_frame;

			return output_frame;
		}

		//TODO: assert that none of the windows are full, because they should all become full at the same time

		return nullptr;
	}

	//-------------------------------------------------------------------------------------

	uint32_t VSFilter::width() const
	{
		return m_CropRegion.width;
	}

	//-------------------------------------------------------------------------------------

	uint32_t VSFilter::height() const
	{
		return m_CropRegion.height;
	}

	//-------------------------------------------------------------------------------------

	Transform VSFilter::clamp_to_frame(const Transform& transform)
	{
		// Reduce the magnitude of the transform until the crop region is
		// fully enclosed within the warped frame. We reduce the magnitude
		// by iteratively lerping the transform back to identity in small steps.

		// Returns identity transform after max iterations
		constexpr int max_iterations = 100;

		double t = 0.0;
		constexpr double max_t = 1.0;
		constexpr double step = max_t/max_iterations;

		auto reduced_transform = transform;
		const auto identity = Transform::Identity();
		while(t <= max_t && !encloses(m_FrameRegion, reduced_transform, m_CropRegion))
		{
			reduced_transform = lerp(transform, identity, t);
			t += step;
		}

		return reduced_transform;
	}

	//-------------------------------------------------------------------------------------

	void VSFilter::configure_sliding_windows(const uint32_t filter_radius)
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
		// and frequency domain performance. An average filter is great in time domain
		// but very bad in the frequency domain performance. Similarly, a windowed sinc
		// filter variation is great in the frequency domain but bad in the time domain
		// with potential to introduce new shaky motion through ringing artifacts.
		// As a rule of thumb, sigma is chosen to fit 99.7% of the distribution in the window.
		const auto gaussian_pdf = cv::getGaussianKernel(full_window_size, full_window_size/6.0);
		for(uint32_t i = 0; i < full_window_size; i++)
			m_PathFilter.push(gaussian_pdf.at<double>(i));

		// Must reset the sliding windows to enforce their synchronisation.
		reset_sliding_windows();
	}

	//-------------------------------------------------------------------------------------

	void VSFilter::reset_sliding_windows()
	{
		// TODO: merge this into an operation with configure?
		// TODO: reset the tracker
		// TODO: properly handle clearing of the OBS frame queue
		// ^^ see https://github.com/obsproject/obs-studio/blob/master/plugins/obs-filters/async-delay-filter.c
		// free_video_data function.

		m_PathWindow.clear();
		m_MotionQueue.clear();
		m_OBSFrameQueue.clear();
		m_StabilisationQueue.clear();

		// Half fill the full sized path window so that the next element gets
		// pushed to its centre, synchronising it with the other half sized buffers.
		for(uint32_t i = 0; i < m_PathWindow.centre_index(); i++)
			m_PathWindow.push(Transform::Identity());
	}

	//-------------------------------------------------------------------------------------

	bool VSFilter::validate() const
	{
		// NOTE: Must run through OpenCL for performance reasons.
		return cv::ocl::haveOpenCL() && cv::ocl::useOpenCL();
	}

	//-------------------------------------------------------------------------------------

}
