#include "VSFilter.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>

namespace lvk
{

	//===================================================================================
	//		CONSTANT PROPERTIES/SETTINGS
	//===================================================================================

	static const FrameTracker::Properties TRACKING_PROPERTIES = {/* Use defaults */};

	//===================================================================================
	//		FILTER IMPLEMENTATION
	//===================================================================================

	obs_properties_t* VSFilter::Properties()
	{

	}

	//-------------------------------------------------------------------------------------

	void VSFilter::LoadDefault(obs_data_t* settings)
	{

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
		// Ingest next frame into the stabilisation system
		m_StabilisationQueue.advance(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY) << next_frame;
		m_OBSFrameQueue.push(next_frame);

		// Track the next frame, and  record the estimated motion & path data
		m_MotionQueue.push(m_FrameTracker.track(m_StabilisationQueue.newest()));
		m_PathWindow.push(m_FrameTracker.cumulative());

		// When all window sizes are satisfied, we can start stabilising the frames in the queue
		if(m_PathWindow.full() && m_MotionQueue.full() && m_OBSFrameQueue.full() && m_StabilisationQueue.full())
		{
			// Apply the motion filter on the path to get the desired smooth path for the frame
			// being processed. We then correct the estimated motion to end on the smooth path.
			const auto smooth_motion = m_PathWindow.convolve(m_PathFilter);
			const auto desired_motion = m_MotionQueue.oldest() + (smooth_motion - m_PathWindow.centre());

			// Clamp the motion to respect the frame crop
			const auto cropped_motion = clamp_to_frame(desired_motion);

			// Perform the stabalisation by warping the frame
			auto& input_frame = m_StabilisationQueue.oldest();
			cv::warpAffine(input_frame, m_WarpFrame, cropped_motion.as_matrix(), input_frame.size());

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

	Transform VSFilter::clamp_to_frame(Transform transform)
	{
	}

	//-------------------------------------------------------------------------------------

	void VSFilter::configure_sliding_windows(const uint32_t filter_radius)
	{
	}

	//-------------------------------------------------------------------------------------

	void VSFilter::reset_sliding_windows()
	{
	}

	//-------------------------------------------------------------------------------------

	bool VSFilter::validate() const
	{
		// NOTE: Must run through OpenCL for performance reasons.
		return cv::ocl::haveOpenCL() && cv::ocl::useOpenCL();
	}

	//-------------------------------------------------------------------------------------

}
