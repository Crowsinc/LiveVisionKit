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

void register_cas_source();

void register_vs_source();

void register_adn_source();

bool obs_module_load()
{
	register_fsr_source();
	register_cas_source();

	// These filters must use OpenCL to run fast enough
	if(cv::ocl::haveOpenCL())
	{
		register_vs_source();
		register_adn_source();
	}

	return true;
}
