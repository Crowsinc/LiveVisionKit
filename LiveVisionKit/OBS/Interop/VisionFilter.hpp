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
#include <queue>
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

		virtual void hybrid_render(gs_texture_t* frame);

	private:

		struct SourceCache
		{
			FrameBuffer frame_buffer;
			uint32_t refs;

			SourceCache();
		};

		bool is_vision_filter_chain_start() const;

		bool is_vision_filter_chain_end() const;

		SourceCache& fetch_cache();
		
		void clean_cache();

		void release_resources();

		bool acquire_frame(FrameBuffer& buffer);

		gs_texture_t* render_frame(FrameBuffer& buffer);

		void prepare_interop(const uint32_t width, const uint32_t height);

	private:

		static std::unordered_map<const obs_source_t*, std::reference_wrapper<VisionFilter>> s_Filters;
		static std::unordered_map<const obs_source_t*, SourceCache> s_SourceCaches;
		static std::mutex s_CacheMutex;
		obs_source_t* m_CacheKey;

		obs_source_t* m_Source;
		obs_source_t* m_Context;
		bool m_Asynchronous, m_HybridRender;

		cv::UMat m_ConversionBuffer;
		gs_texture_t* m_InteropTexture;
		std::queue<obs_source_frame*> m_AsyncFrameQueue; 
	};

}

