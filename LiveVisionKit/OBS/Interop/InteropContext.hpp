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

#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <obs-module.h>

namespace lvk::ocl 
{

	class InteropContext
	{
	public:

		// Creates context
		static bool TryAttach();

		// True if the required OpenCL extensions are supported
		static bool Supported();

		// True if the context is attached to the current thread
		static bool Attached();

		// True if context exists and is usable
		static bool Available();


		static void Import(gs_texture_t* src, cv::UMat& dst);


		static void Export(cv::UMat& src, gs_texture_t* dst);

	private:

		bool static TestContext();

		static cv::ocl::OpenCLExecutionContext s_OCLContext;
		static graphics_t* s_GraphicsContext;
		static std::thread::id s_BoundThread;
		static bool s_TestPassed;

	};

}


