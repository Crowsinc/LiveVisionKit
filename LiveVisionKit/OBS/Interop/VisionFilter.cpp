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
#include "Diagnostics/Assert.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	std::unordered_map<const obs_source_t*, FrameBuffer> VisionFilter::s_FrameCache;
	std::unordered_set<const obs_source_t*> VisionFilter::s_Filters;

//---------------------------------------------------------------------------------------------------------------------

	VisionFilter::VisionFilter(const obs_source_t* filter)
		: m_Context(filter)
	{
		LVK_ASSERT(m_Context != nullptr && s_Filters.count(filter) == 0);

		s_Filters.insert(filter);
	}

//---------------------------------------------------------------------------------------------------------------------

	VisionFilter::~VisionFilter()
	{
		LVK_ASSERT(s_Filters.count(m_Context) == 1);

		s_Filters.erase(m_Context);
	}

//---------------------------------------------------------------------------------------------------------------------

	obs_source_frame* VisionFilter::process(obs_source_frame* input_frame)
	{
		const obs_source* parent = obs_filter_get_parent(m_Context);

		s_FrameCache.try_emplace(parent);
		FrameBuffer& buffer = s_FrameCache[parent];

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
		if(!is_vision_filter_next())
		{
			output_frame = buffer.download();
			buffer.reset();
		}

		return output_frame;
	}

//---------------------------------------------------------------------------------------------------------------------

	const obs_source_t* VisionFilter::find_next_async_filter() const
	{
		using SearchData = std::tuple<const obs_source_t*, bool>;
		SearchData search_data(m_Context, false);

		const auto parent = obs_filter_get_parent(m_Context);
		obs_source_enum_filters(parent, [](obs_source_t* _, obs_source_t* filter, void* search_param){
			auto& [target, found] = *static_cast<SearchData*>(search_param);

			const bool async = (obs_source_get_output_flags(filter) & OBS_SOURCE_ASYNC_VIDEO) == OBS_SOURCE_ASYNC_VIDEO;
			const bool enabled = obs_source_enabled(filter);

			if(!found && filter == target)
				target = nullptr;
			else if(target == nullptr && async && enabled)
			{
				target = filter;
				found = true;
			}
		}, &search_data);

		return std::get<0>(search_data);
	}

//---------------------------------------------------------------------------------------------------------------------

	bool VisionFilter::is_vision_filter_next() const
	{
		const obs_source_t* next_filter = find_next_async_filter();

		return next_filter != nullptr && s_Filters.count(next_filter) > 0;
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
