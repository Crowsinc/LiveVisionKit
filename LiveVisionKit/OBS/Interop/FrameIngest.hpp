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

#pragma once

#include <obs-module.h>
#include <opencv2/opencv.hpp>

namespace lvk
{

	// Direct RAM Frame Ingest

	// Converts OBS frame to YUV UMat
	void import_frame(const obs_source_frame* src, cv::UMat& dst);

	// Converts YUV UMat back to OBS frame, preserves dst alpha channel
	void export_frame(const cv::UMat& src, obs_source_frame* dst);

	// DX11/OpenGL Interop Ingest
	namespace ocl
	{
		// Checks for graphics interop support
		bool supports_graphics_interop();

		// Tries to create an OpenCL context that 
		// is attached to the bound graphics context
		void try_attach_graphics_interop_context();

		// Fills UMat from texture using OpenCL Interop
		void import_texture(gs_texture_t* src, cv::UMat& dst);

		// Fills texture from UMat using OpenCL interop
		void export_texture(cv::UMat& src, gs_texture_t* dst);

	}
}

