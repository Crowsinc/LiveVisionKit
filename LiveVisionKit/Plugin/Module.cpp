#include <obs/obs-module.h>
#include <obs/obs-source.h>
#include <obs/obs.h>

#include <opencv2/core/ocl.hpp>

//=====================================================================================
//		MODULE DECLARATION
//=====================================================================================

OBS_DECLARE_MODULE()

MODULE_EXPORT const char* obs_module_name(void)
{
	return "Live Vision Kit";
}

MODULE_EXPORT const char* obs_module_description(void)
{
	//TODO: re-write this
	return "Live Vision Kit $Description$";
}

//=====================================================================================
//		MODULE SET UP
//=====================================================================================

void register_fsr_source();

void register_vs_source();

void register_test();

bool obs_module_load()
{
	register_fsr_source();

	// Only enable the video stabalisation filter if the user has OpenCL.
	// It will run without it, but it will run too slow to be real-time.
	if(cv::ocl::haveOpenCL())
		register_vs_source();

	register_test();

	return true;
}
