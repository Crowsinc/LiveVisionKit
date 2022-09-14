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

#include <opencv2/core.hpp>
#include <obs-module.h>

#include "FrameIngest.hpp"

namespace lvk
{

	struct FrameBuffer
	{
		cv::UMat frame;
		uint64_t timestamp;

		FrameBuffer();

		~FrameBuffer();

		FrameBuffer(FrameBuffer&& buffer);

		void operator=(FrameBuffer&& buffer);

		uint32_t width() const;
		
		uint32_t height() const;
		
		bool empty() const;

		bool try_upload_frame(obs_source_frame* obs_frame);

		bool try_download_frame(obs_source_frame* obs_frame);

		void import_texture(gs_texture_t* texture);

		void export_texture(gs_texture_t* texture);

	private:

		void prepare_interop_buffer(const uint32_t width, const uint32_t height);

	private:

		// Frame upload/download
		std::unique_ptr<FrameIngest> m_FrameIngest;

		// Texture import/export
		gs_texture_t* m_InteropBuffer = nullptr;
		gs_stagesurf_t* m_ReadBuffer = nullptr;
		gs_texture_t* m_WriteBuffer = nullptr;
		uint8_t* m_MappedData = nullptr;
		cv::UMat m_ConversionBuffer;
	};

}
