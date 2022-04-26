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

#include "VisionFilter.hpp"

#include "Diagnostics/Directives.hpp"

#include <thread>

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	std::unordered_map<const obs_source_t*, VisionFilter::SourceCache> VisionFilter::s_SourceCaches;
	std::unordered_set<const obs_source_t*> VisionFilter::s_Filters;
	std::mutex VisionFilter::s_CacheMutex;

//---------------------------------------------------------------------------------------------------------------------

	VisionFilter::VisionFilter(obs_source_t* filter)
		: m_Filter(filter),
		  m_CacheKey(nullptr)
	{
		LVK_ASSERT(m_Filter != nullptr);
		LVK_ASSERT(s_Filters.count(filter) == 0);

		s_Filters.insert(filter);
	}

//---------------------------------------------------------------------------------------------------------------------

	VisionFilter::~VisionFilter()
	{
		LVK_ASSERT(s_Filters.count(m_Filter) == 1);

		s_Filters.erase(m_Filter);

		clean_cache();
	}

//---------------------------------------------------------------------------------------------------------------------

	void VisionFilter::clean_cache()
	{
		std::scoped_lock cache_lock(s_CacheMutex);

		if (m_CacheKey != nullptr)
		{
			s_SourceCaches[m_CacheKey].refs--;

			if (s_SourceCaches[m_CacheKey].refs == 0)
				s_SourceCaches.erase(m_CacheKey);
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	VisionFilter::SourceCache& VisionFilter::fetch_cache()
	{
		std::scoped_lock cache_lock(s_CacheMutex);
		
		// Lazy initialization of source cache.
		if (m_CacheKey == nullptr)
		{
			// NOTE: this assumes that a filter's parent cannot change in its life time. 
			// This seems to hold true in the OBS source but is not guaranteed in the future. 
			m_CacheKey = obs_filter_get_parent(m_Filter);

			if(s_SourceCaches.count(m_CacheKey) == 0)
				s_SourceCaches.emplace(m_CacheKey, SourceCache{FrameBuffer(), 1});
			else
				s_SourceCaches[m_CacheKey].refs++;
		}

		return s_SourceCaches[m_CacheKey];
	}

//---------------------------------------------------------------------------------------------------------------------

	FrameBuffer& VisionFilter::fetch_frame_cache()
	{
		return fetch_cache().frame_buffer;
	}

//---------------------------------------------------------------------------------------------------------------------

	obs_source_frame* VisionFilter::process(obs_source_frame* input_frame)
	{
		FrameBuffer& buffer = fetch_frame_cache();

		// If this is a new frame, then we need to update the frame cache
		if(buffer != input_frame)
			buffer.upload(input_frame);

		filter(buffer);

		// Frame was captured by the filter (probably to introduce delay).
		if(buffer.empty())
			return nullptr;

		obs_source_frame* output_frame = buffer.handle();

		// If the next filter is not a vision filter, then we need to download the
		// cache back into the OBS frame for the filter. Otherwise, we can just
		// have the next vision filter re-use the frame in the cache.
		if(is_vision_filter_chain_end())
		{
			output_frame = buffer.download();
			buffer.reset();
		}

		return output_frame;
	}

//---------------------------------------------------------------------------------------------------------------------

	void VisionFilter::render()
	{
		if (gs_get_render_target() == nullptr)
		{
			obs_source_skip_video_filter(m_Filter);
			return;
		}

		FrameBuffer& buffer = fetch_frame_cache();

		// If filter is not the start of a vision filter chain, then we want
		// to skip it and travel up the effects filter chain. If it is, then
		// we want to acquire the latest render within the cache buffer and 
		// start travelling down the vision filter chain. If acquisition
		// fails, then continue up the chain as normal. 
		// NOTE: Continuing up the chain may not actually be the correct 
		// solution as filtering may continue on the buffer, which could
		// hold a previous frame in in it. 
		if (!is_vision_filter_chain_start() || !buffer.acquire(m_Filter))
			obs_source_skip_video_filter(m_Filter);
		
		// Perform filtering if the buffer has acquired frame data in it.
		if (!buffer.frame.empty())
		{
			filter(buffer);

			// If this happens to be the last filter in the vision filter
			// chain, then render out the update buffer for the next filters. 
			if (is_vision_filter_chain_end())
				buffer.render();
		}

	}

//---------------------------------------------------------------------------------------------------------------------

	bool VisionFilter::is_vision_filter_chain_start() const
	{
		const obs_source_t* prev_filter = find_prev_filter();

		// We are starting a new vision filter chain if there was no
		// previous filter, or the previous is not a registered vision filter.
		return prev_filter == nullptr || s_Filters.count(prev_filter) == 0;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool VisionFilter::is_vision_filter_chain_end() const
	{
		const obs_source_t* next_filter = find_next_filter();

		// We are ending a vision filter chain if there are no more
		// filters or the next filter is not a registered vision filter.
		return next_filter == nullptr || s_Filters.count(next_filter) == 0;
	}

//---------------------------------------------------------------------------------------------------------------------

	const obs_source_t* VisionFilter::find_prev_filter() const
	{
		using SearchData = std::tuple<const obs_source_t*, const obs_source_t*, uint32_t, bool>;
		SearchData search_data(
			m_Filter,
			nullptr,
			obs_source_get_output_flags(m_Filter) & OBS_SOURCE_ASYNC_VIDEO,
			false
		);

		// Search for the filter right before this filter that is of the same type (sync/async video)
		const auto parent = obs_filter_get_parent(m_Filter);
		obs_source_enum_filters(parent, [](obs_source_t* _, obs_source_t* filter, void* search_param) {
			auto& [target, previous, filter_type, target_found] = *static_cast<SearchData*>(search_param);

			const bool same_type = (obs_source_get_output_flags(filter) & OBS_SOURCE_ASYNC_VIDEO) == filter_type;
			const bool enabled = obs_source_enabled(filter);

			if (!target_found && filter == target)
				target_found = true;
			else if (!target_found && enabled && same_type)
				previous = filter;

			}, &search_data);

		return std::get<1>(search_data);
	}

//---------------------------------------------------------------------------------------------------------------------

	const obs_source_t* VisionFilter::find_next_filter() const
	{
		using SearchData = std::tuple<const obs_source_t*, const obs_source_t*, uint32_t, bool>;
		SearchData search_data(
			m_Filter,
			nullptr,
			obs_source_get_output_flags(m_Filter) & OBS_SOURCE_ASYNC_VIDEO,
			false
		);

		// Search for the next filter after this filter that is of the same type (sync/async video)
		const auto parent = obs_filter_get_parent(m_Filter);
		obs_source_enum_filters(parent, [](obs_source_t* _, obs_source_t* filter, void* search_param){
			auto& [target, next, filter_type, target_found] = *static_cast<SearchData*>(search_param);

			const bool same_type = (obs_source_get_output_flags(filter) & OBS_SOURCE_ASYNC_VIDEO) == filter_type;
			const bool enabled = obs_source_enabled(filter);
			
			if (!target_found && filter == target)
				target_found = true;
			else if (target_found && next == nullptr && enabled && same_type)
				next = filter;

		}, &search_data);

		return std::get<1>(search_data);
	}

//---------------------------------------------------------------------------------------------------------------------

	void VisionFilter::filter(FrameBuffer& buffer)
	{
		filter(buffer.frame);
	}

//---------------------------------------------------------------------------------------------------------------------

	void VisionFilter::filter(cv::UMat& frame) {}

//---------------------------------------------------------------------------------------------------------------------

}
