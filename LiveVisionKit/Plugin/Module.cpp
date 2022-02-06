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

#include <obs/obs-module.h>
#include <opencv2/core/ocl.hpp>

//---------------------------------------------------------------------------------------------------------------------

OBS_DECLARE_MODULE()

//---------------------------------------------------------------------------------------------------------------------

MODULE_EXPORT const char* obs_module_name(void)
{
	return "Live Vision Kit";
}

//---------------------------------------------------------------------------------------------------------------------

void register_fsr_source();

void register_cas_source();

void register_vs_source();

void register_adn_source();

//---------------------------------------------------------------------------------------------------------------------

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

//---------------------------------------------------------------------------------------------------------------------
