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

#include <LiveVisionKit.hpp>
#include <unordered_map>
#include <obs-module.h>
#include <thread>
#include <tuple>
#include <mutex>
#include <deque>

#include "OBSFrame.hpp"

namespace lvk
{

	class VisionFilter
	{
	public:

		explicit VisionFilter(obs_source_t* filter);

		virtual ~VisionFilter();

		obs_source_frame* process(obs_source_frame* frame);

		void render();
		
	protected:

		virtual void filter(OBSFrame& frame) = 0;

		virtual void hybrid_render(gs_texture_t* frame);

		VideoFrame::Format format() const;

		bool is_asynchronous() const;

		Time delta_time() const;

		void disable();

	private:

		struct SourceCache
		{
			OBSFrame frame_buffer;
			uint32_t refs;

			SourceCache();
		};

		bool is_vision_filter_chain_start() const;

		bool is_vision_filter_chain_end() const;

		SourceCache& fetch_cache();
		
		void clean_cache();

		void release_resources();

        void release_async_frames();

		bool acquire_render(OBSFrame& buffer);

		void prepare_render_buffer(const uint32_t width, const uint32_t height);

		obs_source_frame* match_async_frame(OBSFrame& output_buffer, obs_source_frame* input_frame);

	private:
		static std::unordered_map<const obs_source_t*, std::reference_wrapper<VisionFilter>> s_Filters;
		static std::unordered_map<const obs_source_t*, SourceCache> s_SourceCaches;
		static std::mutex s_CacheMutex;
		
		obs_source_t* m_Source = nullptr;
		obs_source_t* m_Context = nullptr;
		obs_source_t* m_CacheKey = nullptr;

		bool m_Asynchronous, m_HybridRender;
		TickTimer m_TickTimer;

		gs_texture_t* m_RenderBuffer = nullptr;
		VideoFrame::Format m_FrameFormat = VideoFrame::UNKNOWN;
		std::deque<std::pair<obs_source_frame*, size_t>> m_AsyncFrameQueue;
        std::thread::id m_GraphicsThread;
	};

}

