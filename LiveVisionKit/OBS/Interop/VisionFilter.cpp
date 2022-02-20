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

#include <util/threading.h>

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

	obs_source_frame* VisionFilter::process(obs_source_frame* frame)
	{
		const obs_source* parent = obs_filter_get_parent(m_Context);

		s_FrameCache.try_emplace(parent);
		FrameBuffer& buffer = s_FrameCache[parent];

		// If this is a new frame, then we are at the start of
		// a filter chain so upload the new frame into the buffer.
		if(buffer != frame)
			buffer.upload(frame);

		filter(buffer);

		// Frame was captured by the filter (probably to introduce delay).
		if(buffer.empty())
			return nullptr;

		// If we are the last vision filter in the chain, then we need to download
		// the buffer back into the OBS frame. If not, the next filter is a Vision
		// filter so we can send OBS the outdated frame but pass the updated frame
		// buffer directly to the next filter.
		if(is_last_in_chain())
		{
			frame = buffer.download();
			buffer.reset();
		}

		return frame;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool VisionFilter::is_last_in_chain() const
	{
		// We are last in the chain if the next filter is disabled or is not a vision filter
		const auto next_filter = obs_filter_get_target(m_Context);
		return !obs_source_enabled(next_filter) || s_Filters.count(next_filter) == 0;
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
