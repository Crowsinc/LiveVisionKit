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

	// Converts OBS frame to YUV UMat
	bool import_frame(const obs_source_frame* src, cv::UMat& dst);

	// Converts YUV UMat back to OBS frame, preserves dst alpha channel
	bool export_frame(const cv::UMat& src, obs_source_frame* dst);
	
	void import_texture(gs_texture_t* src, cv::UMat& dst);

	void export_texture(cv::UMat& src, gs_texture_t* dst);
	
	bool acquire(obs_source_t* source, cv::UMat& frame);

	void release(obs_source_t* source, cv::UMat& frame);

}

