#include <obs/obs-module.h>
#include <obs/obs-source.h>
#include <obs/obs.h>


//================================
//		MODULE DECLARATION
//================================
OBS_DECLARE_MODULE()

//TODO: figure out why this fails to compile
//OBS_MODULE_USE_DEFAULT_LOCALE("LiveVisionKit", "en-US")

MODULE_EXPORT const char* obs_module_name(void)
{
	return "Live Vision Kit";
}

MODULE_EXPORT const char* obs_module_description(void)
{
	//TODO: re-write this
	return "Live Vision Kit Description";
}


//================================
//		MODULE SET UP
//================================

void register_fsr_filter();

//TODO: image stabalisation source
//TODO: use an ASYNC video output in order to access frame
// data from RAM so that it can be used along with OpenCV
//void register_is_source();

bool obs_module_load()
{
	register_fsr_filter();

	// Return false on fail
	return true;
}
