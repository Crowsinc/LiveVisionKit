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
#include <obs-source.h>
#include <obs.h>

namespace lvk
{

	struct FrameBuffer
	{
		cv::UMat frame;

		FrameBuffer();

		FrameBuffer(FrameBuffer&& buffer);

		FrameBuffer(const FrameBuffer& buffer) = delete;

		void reset();

		void release(obs_source_t* owner);

		bool empty() const;

		obs_source_frame* download() const;

		void upload(obs_source_frame* source_frame);

		bool operator==(const FrameBuffer& other) const;

		bool operator==(obs_source_frame* source_frame) const;

		bool operator!=(const FrameBuffer& other) const;

		bool operator!=(obs_source_frame* source_frame) const;

	private:
		obs_source_frame* m_SourceFrame;
	};

}
