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

#include <unordered_map>
#include <unordered_set>

#include <opencv2/core.hpp>
#include <obs-module.h>

#include "FrameBuffer.hpp"

namespace lvk
{

	class VisionFilter
	{
	public:

		VisionFilter(const obs_source_t* filter);

		virtual ~VisionFilter();

		obs_source_frame* process(obs_source_frame* frame);

	protected:

		virtual void filter(FrameBuffer& buffer);

		virtual void filter(cv::UMat& frame);

	private:

		static std::unordered_map<const obs_source_t*, FrameBuffer> s_FrameCache;
		static std::unordered_set<const obs_source_t*> s_Filters;

		const obs_source_t* m_Context;

		static void on_filter_remove(void* data, calldata_t* call_data);

		const obs_source_t* find_next_async_filter() const;

		bool is_vision_filter_next() const;

	};

}
