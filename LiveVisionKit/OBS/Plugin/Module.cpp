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

#include <obs-module.h>
#include <opencv2/core/ocl.hpp>

#include "Diagnostics/Directives.hpp"
#include "OBS/Interop/FrameIngest.hpp"

//---------------------------------------------------------------------------------------------------------------------

OBS_DECLARE_MODULE()

//---------------------------------------------------------------------------------------------------------------------

MODULE_EXPORT const char* obs_module_name(void)
{
	return "LiveVisionKit v1.1.1";
}

//---------------------------------------------------------------------------------------------------------------------

void register_fsr_source();

void register_cas_source();

void register_vs_source();

void register_adb_source();

void register_adb_effect_source();

void register_lc_source();

void register_cct_source();

//---------------------------------------------------------------------------------------------------------------------

void attach_ocl_interop_context(void* param, uint32_t cx, uint32_t cy)
{
	// NOTE: Attach (create) OpenCV's OpenCL context based on the graphics
	// context in use by OBS. Do it here before any OpenCL code is able to
	// run, then immediately remove the callback.
	lvk::ocl::try_attach_graphics_interop_context();
	obs_remove_main_render_callback(&attach_ocl_interop_context, nullptr);
}

//---------------------------------------------------------------------------------------------------------------------

bool obs_module_load()
{
	register_fsr_source();
	register_cas_source();

	// Vision Async Filters 
	if(cv::ocl::haveOpenCL())
	{
		register_lc_source();
		register_vs_source();
		register_adb_source();

		register_cct_source();
	}
	else LVK_ERROR("OpenCL unsupported");

	// Vision Effects Filters
	if (lvk::ocl::supports_graphics_interop())
	{
		obs_add_main_render_callback(&attach_ocl_interop_context, nullptr);
	}
	else LVK_ERROR("OpenCL Interop unsupported");

	return true;
}

//---------------------------------------------------------------------------------------------------------------------
