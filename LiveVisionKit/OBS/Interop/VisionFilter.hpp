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

#include <mutex>
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

		VisionFilter(obs_source_t* filter);

		virtual ~VisionFilter();

		obs_source_frame* process(obs_source_frame* frame);

		void render();

	protected:

		virtual void filter(FrameBuffer& buffer);

		virtual void filter(cv::UMat& frame);

	private:

		struct SourceCache
		{
			FrameBuffer frame_buffer;
			uint32_t refs;
		};

		const obs_source_t* find_next_filter() const;

		const obs_source_t* find_prev_filter() const;

		bool is_vision_filter_chain_start() const;

		bool is_vision_filter_chain_end() const;

		void clean_cache();

		SourceCache& fetch_cache();

		FrameBuffer& fetch_frame_cache();

	private:

		static std::unordered_map<const obs_source_t*, SourceCache> s_SourceCaches;
		static std::unordered_set<const obs_source_t*> s_Filters;
		static std::mutex s_CacheMutex;

		obs_source_t* m_Filter;
		obs_source_t* m_CacheKey;
	};

}
