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

#include "Utility/ScopedProfiler.hpp"
#include "Effects/DefaultEffect.hpp"
#include "Utility/Graphics.hpp"
#include "Utility/Logging.hpp"
#include "FrameIngest.hpp"

#include <util/platform.h>

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

    constexpr size_t MAX_FRAME_SKIPS = 15;

//---------------------------------------------------------------------------------------------------------------------
	
	std::unordered_map<const obs_source_t*, VisionFilter::SourceCache> VisionFilter::s_SourceCaches;
	std::unordered_map<const obs_source_t*, std::reference_wrapper<VisionFilter>> VisionFilter::s_Filters;
	std::mutex VisionFilter::s_CacheMutex;

//---------------------------------------------------------------------------------------------------------------------

	VisionFilter::VisionFilter(obs_source_t* context)
		: m_Context(context),
		m_Asynchronous(test_bits<uint32_t>(
			obs_source_get_output_flags(context),
			OBS_SOURCE_ASYNC_VIDEO
			)),
		// NOTE: We initially assume a hybrid render state for each filter, 
		// then update our assumption as we learn more about them during execution.
        // - Synchronous filters are assumed to be hybrid render, but the assumption
        //   is revoked if their hybrid_render() function is not implemented.
        // - Asynchrnous filters are not hybrid render by default, and only become
        //   if their hybrid_render function has been implemented.
		m_HybridRender(!m_Asynchronous)
	{
		LVK_ASSERT(m_Context != nullptr);
		LVK_ASSERT(s_Filters.count(context) == 0);

		s_Filters.emplace(context, std::ref(*this));

        m_TickTimer.start();
	}

//---------------------------------------------------------------------------------------------------------------------

	VisionFilter::~VisionFilter()
	{
		LVK_ASSERT(s_Filters.count(m_Context) == 1);

		s_Filters.erase(m_Context);

		clean_cache();
		release_resources();
	}

//---------------------------------------------------------------------------------------------------------------------

	void VisionFilter::release_resources()
	{
		while(m_Source != nullptr && !m_AsyncFrameQueue.empty())
		{
			obs_source_release_frame(m_Source, m_AsyncFrameQueue.front().first);
			m_AsyncFrameQueue.pop_front();
		}

		obs_enter_graphics();

		if(m_RenderBuffer != nullptr)
		{
			gs_texture_destroy(m_RenderBuffer);
			m_RenderBuffer = nullptr;
		}

		obs_leave_graphics();
	}

//---------------------------------------------------------------------------------------------------------------------

	void VisionFilter::clean_cache()
	{
		std::scoped_lock cache_lock(s_CacheMutex);

		if(m_CacheKey != nullptr)
		{
			s_SourceCaches[m_CacheKey].refs--;

			if(s_SourceCaches[m_CacheKey].refs == 0)
			{
				log::warn("Releasing filter cache for \'%s\'", obs_source_get_name(m_CacheKey));
				s_SourceCaches.erase(m_CacheKey);
			}
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	VisionFilter::SourceCache& VisionFilter::fetch_cache()
	{
		std::scoped_lock cache_lock(s_CacheMutex);

		// Lazy initialization of source cache.
		if(m_CacheKey == nullptr)
		{
			// NOTE: this assumes that a filter's parent cannot change in its life time. 
			// This seems to hold true in the OBS source but is not guaranteed in the future. 
			m_CacheKey = obs_filter_get_parent(m_Context);

			if(s_SourceCaches.count(m_CacheKey) == 0)
				s_SourceCaches.emplace(m_CacheKey, SourceCache{});
			else
				s_SourceCaches[m_CacheKey].refs++;
		}

		return s_SourceCaches[m_CacheKey];
	}

//---------------------------------------------------------------------------------------------------------------------

	obs_source_frame* VisionFilter::process(obs_source_frame* input_frame)
	{
        LVK_PROFILE;

		m_Source = obs_filter_get_parent(m_Context);
		if(m_Source == nullptr || input_frame == nullptr)
			return input_frame;

		auto& buffer = fetch_cache().frame_buffer;

		// If we are at the start of a new chain, upload the frame to the frame buffer. 
		// If the upload fails, because the frame format isn't supported, then we will
		// disable this filter and pass the given frame down the filter chain.
		if(is_vision_filter_chain_start())
		{
			if(!buffer.try_upload_frame(input_frame))
			{
				log::error(
					"\'%s\' was applied on an unsupported video stream (%s), disabling the filter...",
					obs_source_get_name(m_Context),
					get_video_format_name(input_frame->format)
				);

				disable();
				return input_frame;
			}
		}

		m_TickTimer.tick();
		filter(buffer);

		auto output_frame = match_async_frame(buffer, input_frame);
		if(output_frame == nullptr)
			return nullptr;

		// If the next filter is not a vision filter, then we need to save the
		// frame buffer back into the OBS frame for the non-vision filter.
		if(is_vision_filter_chain_end())
		{
			if(!buffer.try_download_frame(output_frame))
			{
				log::error(
					"\'%s\' tried to download its frame buffer to an unsupported video stream! (%s)",
					obs_source_get_name(m_Context),
					get_video_format_name(output_frame->format)
				);
			}
		}

		return output_frame;
	}

//---------------------------------------------------------------------------------------------------------------------

	obs_source_frame* VisionFilter::match_async_frame(FrameBuffer& output_buffer, obs_source_frame* input_frame)
	{
        LVK_PROFILE;

		obs_source_frame* output_frame = nullptr;

        // If there is no output buffer, then we are just building delay.
        if(output_buffer.is_empty())
        {
            m_AsyncFrameQueue.emplace_back(input_frame, 0);
            return nullptr;
        }

        // If the output buffer is already matched with the input frame then we
        // can directly return. This will be the case for all zero delay filters.
        // Otherwise, we need to go through the full matching and clean up process.
        if(m_AsyncFrameQueue.empty() && output_buffer.timestamp == input_frame->timestamp)
        {
            return input_frame;
        }
        else m_AsyncFrameQueue.emplace_back(input_frame, 0);

        // Attempt to match the output buffer to an existing frame in the queue. The
        // frame queue is in the same order that the filter is applied to the frames,
        // so any frames which come before the match are assumed to be skipped. Once
        // a frame has been skipped too many times it will be deallocated and removed.
        size_t released_frames = 0;
        for(auto iter = m_AsyncFrameQueue.begin(); iter != m_AsyncFrameQueue.end(); ++iter)
        {
            obs_source_frame* frame = iter->first;
            size_t& skips = iter->second;

            if(frame->timestamp == output_buffer.timestamp)
            {
                // Frame was matched
                output_frame = frame;
                m_AsyncFrameQueue.erase(iter);
                break;
            }

            if(skips++ == MAX_FRAME_SKIPS)
            {
                // Frame needs to be released
                m_AsyncFrameQueue.erase(iter);
                obs_source_release_frame(m_Source, frame);
                released_frames++;
            }
        }

        log::warn_if(
            released_frames > 0,
            "\'%s\' released %d frames.",
            obs_source_get_name(m_Context),
            released_frames
        );

        log::error_if(
            output_frame == nullptr,
            "\'%s\' failed to find a matching frame!",
            obs_source_get_name(m_Context)
        );

		return output_frame;
	}

//---------------------------------------------------------------------------------------------------------------------

	void VisionFilter::render()
	{
		LVK_ASSERT(gs_get_context() != nullptr);
        LVK_PROFILE;

		m_Source = obs_filter_get_parent(m_Context);
		if(m_Source == nullptr)
			return;

		// The render target will be nullptr if we are the last effect filter
		// and OBS is attempting to render the filter preview window. Assuming
		// this is true, we can avoid re-rendering the filter by rendering the
		// render buffer, which must contain the most up to date frame because
		// we are the last filter in the chain.
		if(gs_get_render_target() == nullptr)
		{
			hybrid_render(m_RenderBuffer);
			return;
		}

		// All asynchronous vision filters which are configured with a render()
		// are hybrid render filters by definition, and should be handled here.
		if(m_Asynchronous)
		{
			m_HybridRender = true;
			hybrid_render(nullptr);

			// m_HybridRender is set to false if hybrid_render() was not 
			// properly overwritten, meaning the filter is misconfigured. 
			LVK_ASSERT(m_HybridRender);
			return;
		}

		FrameBuffer& buffer = fetch_cache().frame_buffer;
		bool is_chain_start = false, is_chain_end = false;

		// Render to the frame buffer if we are at the start of a new chain,
		// otherwise pretend to skip the filter so that OBS travels up the
		// filter chain to process previous effects filters.
		if(is_chain_start = is_vision_filter_chain_start(); is_chain_start)
		{
			// If rendering to the frame buffer somehow fails, then release
            // the buffer so that upcoming filters don't try to filter an
            // outdated frame. This shouldn't occur during normal operation.
			if(!acquire_render(buffer))
			{
				buffer.data.release();
				obs_source_skip_video_filter(m_Context);

				log::warn(
					"\'%s\' failed to acquire the current frame!",
					obs_source_get_name(m_Context)
				);
			}
		}
		else obs_source_skip_video_filter(m_Context);

		// Here we are travelling back down the filter chain so 
		// perform filtering on the buffer's captured frame, if any. 
		if(!buffer.is_empty())
		{
			m_TickTimer.tick();
			filter(buffer);

			// Frame was captured by the filter (probably to introduce delay).
			if(buffer.is_empty())
				return;

			// If this is the last filter in the vision filter chain,
			// then render out the buffer for the non-vision filters. 
			if(is_chain_end = is_vision_filter_chain_end(); is_chain_end)
			{
				prepare_render_buffer(buffer.width(), buffer.height());
				buffer.export_texture(m_RenderBuffer);
				hybrid_render(m_RenderBuffer);
			}
		}

		// Clean up buffers if we are not at the chain ends
		if(!is_chain_start && !is_chain_end && m_RenderBuffer != nullptr)
		{
			gs_texture_destroy(m_RenderBuffer);
			m_RenderBuffer = nullptr;
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	bool VisionFilter::is_vision_filter_chain_start() const
	{
		LVK_ASSERT(m_Source != nullptr);

		using SearchState = std::tuple<
			bool, // Search result
			bool, // Search runflag
			const VisionFilter* // Reference filter
		>;

		// NOTE: Search result defaults to starting a chain by default
		SearchState search_state(true, true, this);

		obs_source_enum_filters(m_Source, [](auto parent, auto curr_filter, void* state) {

			auto& [start_chain, runflag, ref_filter] = *static_cast<SearchState*>(state);

			// Determine whether the reference filter is starting a new chain, by testing
			// the necessary conditions against all filters before the reference filter.  
			// The tests end once we find the reference filter, meaning that only most
			// recent test, corresponding to the previous filter, is returned at the end.

			// Deactivate search once we reach the reference filter.
			if(curr_filter == ref_filter->m_Context)
				runflag = false;

			if(runflag && obs_source_enabled(curr_filter))
			{
				const auto flags = obs_source_get_output_flags(curr_filter);

				const bool is_asynchronous = test_bits<uint32_t>(flags, OBS_SOURCE_ASYNC_VIDEO);
				const bool is_same_type = ref_filter->m_Asynchronous == is_asynchronous;

				const bool is_vision_filter = s_Filters.count(curr_filter) != 0;
				const bool is_hybrid_render = is_vision_filter && s_Filters.at(curr_filter).get().m_HybridRender;

				if(is_same_type)
				{
					// Start chain if the previous filter is not a vision filter or we
					// are both synchronous, but the previous filter is hybrid render.
					start_chain = !is_vision_filter || (!is_asynchronous && is_hybrid_render);
				}
				else if(!is_same_type && is_asynchronous && is_hybrid_render)
				{
					// Always start a new chain if we are synchronous and the 
					// previous filter is an asynchronous hybrid render filter
					start_chain = true;
				}
			}
        }, &search_state);

		return std::get<0>(search_state);
	}

//---------------------------------------------------------------------------------------------------------------------

	bool VisionFilter::is_vision_filter_chain_end() const
	{
		LVK_ASSERT(m_Source != nullptr);

		using SearchState = std::tuple<
			bool, // Search result
			bool, // Search runflag
			const VisionFilter* // Reference filter
		>;

		// Always end the chain if we have synchronous hybrid render
		if(m_HybridRender && !m_Asynchronous)
			return true;

		// NOTE: Search result defaults to ending the chain by default
		SearchState search_state(true, false, this);

		obs_source_enum_filters(m_Source, [](auto parent, auto curr_filter, void* state) {

			auto& [end_chain, runflag, ref_filter] = *static_cast<SearchState*>(state);

			if(runflag && obs_source_enabled(curr_filter))
			{
				const auto flags = obs_source_get_output_flags(curr_filter);

				const bool is_asynchronous = test_bits<uint32_t>(flags, OBS_SOURCE_ASYNC_VIDEO);
				const bool is_same_type = ref_filter->m_Asynchronous == is_asynchronous;

				const bool is_vision_filter = s_Filters.count(curr_filter) != 0;
				const bool is_hybrid_render = is_vision_filter && s_Filters.at(curr_filter).get().m_HybridRender;

				// First filter of the same type is the next filter
				if(is_same_type)
				{
					// End chain if next filter is not another vision filter
					end_chain = !is_vision_filter;
					runflag = false;
				}
				else if(!is_same_type && is_asynchronous && is_hybrid_render)
				{
					// Always end the chain if we are synchronous and the 
					// next filter is an asynchronous hybrid render filter
					end_chain = true;
					runflag = false;
				}
			}

			// Only activate the search once we reach the reference filter
			if(curr_filter == ref_filter->m_Context)
				runflag = true;

			}, &search_state);

		return std::get<0>(search_state);
	}

//---------------------------------------------------------------------------------------------------------------------

	bool VisionFilter::acquire_render(FrameBuffer& buffer)
	{
        LVK_PROFILE;

		const auto target = obs_filter_get_target(m_Context);
		const uint32_t source_width = obs_source_get_base_width(target);
		const uint32_t source_height = obs_source_get_base_height(target);

		if(target == nullptr || source_width == 0 || source_height == 0)
			return false;

		prepare_render_buffer(source_width, source_height);
		if(DefaultEffect::Acquire(m_Context, m_RenderBuffer))
		{
			buffer.import_texture(m_RenderBuffer);
			buffer.timestamp = os_gettime_ns();
			return true;
		}
		return false;
	}

//---------------------------------------------------------------------------------------------------------------------

	void VisionFilter::prepare_render_buffer(const uint32_t width, const uint32_t height)
	{
		prepare_texture(
			m_RenderBuffer,
			width,
			height,
			GS_RGBA,
			GS_RENDER_TARGET
		);
	}

//---------------------------------------------------------------------------------------------------------------------

	void VisionFilter::disable()
	{
		obs_source_set_enabled(m_Context, false);
		release_resources();
	}

//---------------------------------------------------------------------------------------------------------------------

	Time VisionFilter::delta_time() const
	{
		return m_TickTimer.delta();
	}

//---------------------------------------------------------------------------------------------------------------------

	void VisionFilter::filter(FrameBuffer& buffer)
	{
		filter(buffer.data);
	}

//---------------------------------------------------------------------------------------------------------------------

	void VisionFilter::filter(cv::UMat& frame) {}

//---------------------------------------------------------------------------------------------------------------------

	void VisionFilter::hybrid_render(gs_texture_t* frame)
	{
		LVK_ASSERT(gs_get_context() != nullptr);

		// Filter is not hybrid render if this function has not been overwritten.
		m_HybridRender = false;

		if(frame != nullptr)
			DefaultEffect::Render(frame);
		else
			DefaultEffect::Render(m_Context);
	}
	
//---------------------------------------------------------------------------------------------------------------------

	VisionFilter::SourceCache::SourceCache()
		: refs(1)
	{}
	
//---------------------------------------------------------------------------------------------------------------------

}
