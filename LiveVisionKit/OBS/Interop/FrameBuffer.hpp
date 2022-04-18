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

namespace lvk
{

	struct FrameBuffer
	{
		cv::UMat frame;

		FrameBuffer();

		FrameBuffer(FrameBuffer&& buffer);

		FrameBuffer(const FrameBuffer& buffer) = delete;

		~FrameBuffer();

		void reset();

		void release(obs_source_t* owner);

		bool empty() const;

		uint64_t timestamp() const;

		obs_source_frame* handle() const;

		void upload(obs_source_frame* frame_handle);

		obs_source_frame* download() const;

		bool acquire(const obs_source_t* source);

		void render();

		void operator=(FrameBuffer&& buffer);

		bool operator==(const FrameBuffer& other) const;

		bool operator==(obs_source_frame* frame_handle) const;

		bool operator!=(const FrameBuffer& other) const;

		bool operator!=(obs_source_frame* frame_handle) const;

	private:

		void prepare_interop_texture(const uint32_t width, const uint32_t height);

	private:

		obs_source_frame* m_FrameHandle;

		gs_texture_t* m_InteropTexture;
		cv::UMat m_InteropBuffer;
	};

}
