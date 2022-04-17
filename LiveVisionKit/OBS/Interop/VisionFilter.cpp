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

#include "FrameIngest.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	constexpr auto FILTER_REMOVE_SIGNAL = "filter_remove";

//---------------------------------------------------------------------------------------------------------------------

	std::unordered_map<const obs_source_t*, FrameBuffer> VisionFilter::s_FrameCache;
	std::unordered_set<const obs_source_t*> VisionFilter::s_Filters;

//---------------------------------------------------------------------------------------------------------------------

	void VisionFilter::on_filter_remove(void* data, calldata_t* call_data)
	{
		obs_source_t* source = static_cast<obs_source_t*>(calldata_ptr(call_data, "source"));
		obs_source_t* removed_filter = static_cast<obs_source_t*>(calldata_ptr(call_data, "filter"));

		// Source Cache Clean Up

		using SearchData = std::tuple<std::unordered_set<const obs_source_t*>*, const obs_source_t*, bool>;
		SearchData search_data(&s_Filters, removed_filter, false);

		// Go through the source's filters to try and find another vision filter
		obs_source_enum_filters(source, [](obs_source_t* _, obs_source_t* filter, void* search_param){
			auto& [filters, remove_filter, found] = *static_cast<SearchData*>(search_param);

			if(filter != remove_filter && filters->count(filter) == 1)
				found = true;

		}, &search_data);

		// Remove cache if the source has no other vision filters
		if(!std::get<2>(search_data))
		{
			s_FrameCache.erase(source);
			signal_handler_disconnect(
				obs_source_get_signal_handler(source),
				FILTER_REMOVE_SIGNAL,
				&on_filter_remove,
				nullptr
			);
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	VisionFilter::VisionFilter(obs_source_t* filter)
		: m_Context(filter)
	{
		LVK_ASSERT(m_Context != nullptr);
		LVK_ASSERT(s_Filters.count(filter) == 0);

		s_Filters.insert(filter);

		// NOTE: Initialize the interop context here because this is the most 
		// convenient spot runs before any OpenCL is used, and after the OBS
		// creates the graphics context. 
		try_initialize_interop_context();
	}

//---------------------------------------------------------------------------------------------------------------------

	VisionFilter::~VisionFilter()
	{
		LVK_ASSERT(s_Filters.count(m_Context) == 1);

		s_Filters.erase(m_Context);
	}

//---------------------------------------------------------------------------------------------------------------------

	FrameBuffer& VisionFilter::fetch_cache()
	{
		const obs_source_t* parent = obs_filter_get_parent(m_Context);

		// Register any new parents
		if (s_FrameCache.count(parent) == 0)
		{
			s_FrameCache.emplace(parent, FrameBuffer());
			signal_handler_connect(
				obs_source_get_signal_handler(parent),
				FILTER_REMOVE_SIGNAL,
				&on_filter_remove,
				nullptr
			);
		}

		return s_FrameCache[parent];
	}

//---------------------------------------------------------------------------------------------------------------------

	obs_source_frame* VisionFilter::process(obs_source_frame* input_frame)
	{
		FrameBuffer& buffer = fetch_cache();

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
		FrameBuffer& buffer = fetch_cache();

		// If this is the first effect vision filter in a chain, 
		// then render the source into our frame cache
		if (is_vision_filter_chain_start())
			buffer.acquire(m_Context);

		filter(buffer);

		// If the frame buffer wasn't captured by the filter and this is the 
		// last vision filter in the chain. Then we need to render the frame
		// buffer out to OBS so that the next filter can operate on the updated
		// frame. Otherwise, we just skip rendering and have the next vision 
		// filter re-use the frame in the cache.
		if(!buffer.empty() && is_vision_filter_chain_end())
			buffer.render();
		else
			obs_source_skip_video_filter(m_Context);
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
			m_Context,
			nullptr,
			obs_source_get_output_flags(m_Context) & OBS_SOURCE_ASYNC_VIDEO,
			false
		);

		// Search for the filter right before m_Context that is of the same type (sync/async video)

		const auto parent = obs_filter_get_parent(m_Context);
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
			m_Context,
			nullptr,
			obs_source_get_output_flags(m_Context) & OBS_SOURCE_ASYNC_VIDEO,
			false
		);

		// Search for the next filter after m_Context that is of the same type (sync/async video)

		const auto parent = obs_filter_get_parent(m_Context);
		obs_source_enum_filters(parent, [](obs_source_t* _, obs_source_t* filter, void* search_param){
			auto& [target, next, filter_type, target_found] = *static_cast<SearchData*>(search_param);

			const bool same_type = (obs_source_get_output_flags(filter) & OBS_SOURCE_ASYNC_VIDEO) == filter_type;
			const bool enabled = obs_source_enabled(filter);
			
			if (!target_found && filter == target)
				target_found = true;
			else if(target_found && enabled && same_type)
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
