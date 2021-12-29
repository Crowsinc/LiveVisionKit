#include <obs/obs-module.h>
#include <obs/obs-source.h>
#include <obs/obs.h>

//================================
//		MODULE DECLARATION
//================================

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

//================================
//		MODULE SET UP
//================================

void register_fsr_plugin();



bool obs_module_load()
{
	register_fsr_plugin();

	return true;
}
