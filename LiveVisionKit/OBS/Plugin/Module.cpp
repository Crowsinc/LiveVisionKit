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
#include "OBS/Utility/Logging.hpp"

#include "OBS/Effects/FSREffect.hpp"
#include "OBS/Effects/CASEffect.hpp"

//---------------------------------------------------------------------------------------------------------------------

#define VERSION "1.2.0"

//---------------------------------------------------------------------------------------------------------------------

OBS_DECLARE_MODULE()

//---------------------------------------------------------------------------------------------------------------------

MODULE_EXPORT const char* obs_module_name(void)
{
	return "LiveVisionKit" VERSION;
}

//---------------------------------------------------------------------------------------------------------------------

void register_fsr_source();

void register_cas_source();

void register_vs_source();

void register_vs_effect_source();

void register_adb_source();

void register_adb_effect_source();

void register_lc_source();

void register_lc_effect_source();

void register_cct_source();

void register_cct_effect_source();

//---------------------------------------------------------------------------------------------------------------------

void attach_ocl_interop_context(void* param, uint32_t cx, uint32_t cy)
{
	// NOTE: We need to attach (create) the OpenCL context based on the graphics
	// context in use by OBS. We need to do it here during the main render so 
	// that it occurs before any OpenCL vision filter code is able to run. We 
	// also need to attempt it repeatedly in case OBS switches to a new graphics
	// render thread. If this happens, then our OpenCL execution context will be
	// attached to the wrong thread, and must be updated before running OpenCL code.
	lvk::ocl::try_attach_graphics_interop_context();
}

//---------------------------------------------------------------------------------------------------------------------

bool obs_module_load()
{
	// Detect LVK capabilities
	const bool has_opencl = cv::ocl::haveOpenCL();
	const bool has_interop = lvk::ocl::supports_graphics_interop();
	const bool has_fsr_effect = lvk::FSREffect::Validate();
	const bool has_cas_effect = lvk::CASEffect::Validate();
	
	lvk::log::print_block(
		"Initializing..."
		"\n    Version: %s"
		"\n    OpenCL Support: %s"
		"\n    Interop Support: %s"
		"\n    FSR Effect Loaded: %s"
		"\n    CAS Effect Loaded: %s"
		,
		VERSION,
		has_opencl ? "Yes" : "No",
		has_interop ? "Yes" : "No",
		has_fsr_effect ? "Yes" : "No",
		has_cas_effect ? "Yes" : "No"
	);

	// Attach OpenCL context
	if(has_interop)
		obs_add_main_render_callback(&attach_ocl_interop_context, nullptr);

	// Register Filters...
	register_fsr_source();
	register_cas_source();

	if(has_opencl)
	{
		register_vs_source();
		register_lc_source();
		register_adb_source();
		register_cct_source();
		
		register_vs_effect_source();
		register_adb_effect_source();
		register_lc_effect_source();
		register_cct_effect_source();
	}

	return true;
}

//---------------------------------------------------------------------------------------------------------------------
