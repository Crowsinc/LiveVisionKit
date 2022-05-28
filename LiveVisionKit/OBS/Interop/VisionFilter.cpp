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

#include "FrameIngest.hpp"
#include "OBS/Effects/DefaultEffect.hpp"
#include "Diagnostics/Directives.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	std::unordered_map<const obs_source_t*, VisionFilter::SourceCache> VisionFilter::s_SourceCaches;
	std::unordered_map<const obs_source_t*, std::reference_wrapper<VisionFilter>> VisionFilter::s_Filters;
	std::mutex VisionFilter::s_CacheMutex;

//---------------------------------------------------------------------------------------------------------------------

	VisionFilter::VisionFilter(obs_source_t* context)
		: m_CacheKey(nullptr),
		  m_Source(nullptr),
		  m_Context(context),
		  m_Asynchronous(test_bits<uint32_t>(
			  obs_source_get_output_flags(context),
			  OBS_SOURCE_ASYNC_VIDEO
		  )),
		  // NOTE: We initially assume a hybrid render state for each filter, 
		  // then update our assumption as we learn more about them during execution. 
		  m_HybridRender(m_Asynchronous ? false : true),
		  m_InteropBuffer(nullptr),
		  m_RenderBuffer(nullptr),
		  m_ConversionBuffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY)
	{
		LVK_ASSERT(m_Context != nullptr);
		LVK_ASSERT(s_Filters.count(context) == 0);

		s_Filters.emplace(context, std::ref(*this));
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
		while (m_Source != nullptr && !m_AsyncFrameQueue.empty())
		{
			obs_source_release_frame(m_Source, m_AsyncFrameQueue.front());
			m_AsyncFrameQueue.pop();
		}

		obs_enter_graphics();

		if(m_RenderBuffer != nullptr)
			gs_texture_destroy(m_RenderBuffer);

		if(m_InteropBuffer != nullptr)
			gs_texture_destroy(m_InteropBuffer);

		obs_leave_graphics();

		m_ConversionBuffer.release();
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
		m_Source = obs_filter_get_parent(m_Context);
		if(m_Source == nullptr)
			return input_frame;

		auto& buffer = fetch_cache().frame_buffer;

		// Load the frame to the frame buffer if we are at the start of a new chain.
		if(is_vision_filter_chain_start())
			buffer.load(input_frame);

		filter(buffer);
		
		// Frame was captured by the filter (probably to introduce delay).
		m_AsyncFrameQueue.push(input_frame);
		if (buffer.empty())
			return nullptr;
		
		// Release any skipped past frames to avoid memory leaks.
		// The filter must output frames in chronological order.
		while (buffer.timestamp > m_AsyncFrameQueue.front()->timestamp)
		{
			obs_source_release_frame(m_Source, m_AsyncFrameQueue.front());
			m_AsyncFrameQueue.pop();
		}

		// After removing past frames, the front of the queue
		// must have the frame which corresponds to the buffer.
		LVK_ASSERT(m_AsyncFrameQueue.front()->timestamp == buffer.timestamp);
		obs_source_frame* output_frame = m_AsyncFrameQueue.front();
		m_AsyncFrameQueue.pop();

		// If the next filter is not a vision filter, then we need to save the
		// frame buffer back into the OBS frame for the non-vision filter.
		if(is_vision_filter_chain_end())
			buffer.save(output_frame);

		return output_frame;
	}

//---------------------------------------------------------------------------------------------------------------------

	void VisionFilter::render()
	{
		LVK_ASSERT(gs_get_context() != nullptr);

		m_Source = obs_filter_get_parent(m_Context);
		if(m_Source == nullptr || gs_get_render_target() == nullptr)
		{
			obs_source_skip_video_filter(m_Context);
			return;
		}

		if (m_Asynchronous)
		{
			// An asynchronous filter must be hybrid render for it to find itself in this
			// scope, so update our assumption. If this assumption doesn't hold through 
			// the hybrid_render call, because it has not been overwritten, then we 
			// consider this asynchronous vision filter to be misconfigured.
			m_HybridRender = true;
			hybrid_render(nullptr);
			LVK_ASSERT(m_HybridRender);
			return;
		}

		FrameBuffer& buffer = fetch_cache().frame_buffer;
		bool is_chain_start = false, is_chain_end = false;

		// Render to the frame buffer if we are at the start of a new chain,
		// otherwise pretend to skip the filter so that OBS travels up the
		// filter chain to process previous effects filters.
		if (is_chain_start = is_vision_filter_chain_start(); is_chain_start)
		{
			// If rendering to the frame buffer somehow fails, then
			// release the buffer so that upcoming filters don't try
			// and filter and an outdated frame. This should rarely
			// occur during normal operation.
			if (!acquire_render(buffer))
			{
				buffer.frame.release();
				obs_source_skip_video_filter(m_Context);
				LVK_WARN("Interop failed to acquire frame");
			}
		}
		else obs_source_skip_video_filter(m_Context);

		// Here we are travelling back down the filter chain so 
		// perform filtering on the buffer's captured frame, if any. 
		if (!buffer.empty())
		{
			filter(buffer);

			// Frame was captured by the filter (probably to introduce delay).
			if (buffer.empty())
				return;

			// If this happens to be the last filter in the vision filter
			// chain, then render out the buffer for the non-vision filters. 
			if (is_chain_end = is_vision_filter_chain_end(); is_chain_end)
				hybrid_render(acquire_buffer(buffer));
		}

		// Clean up interop resources if we are not at the chain ends
		// Both require interop buffer, start might need render buffer
		if (!is_chain_start)
		{
			if (m_RenderBuffer != nullptr)
			{
				gs_texture_destroy(m_RenderBuffer);
				m_RenderBuffer = nullptr;
			}

			if (!is_chain_end && m_InteropBuffer != nullptr)
			{
				gs_texture_destroy(m_InteropBuffer);
				m_InteropBuffer = nullptr;

				m_ConversionBuffer.release();
			}
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
		
			// Determine the chain start state for all filters before the reference filter.
			// The latest state once we reach the reference filter will belong to the 
			// previous filter that we are after. 

			// Deactivate search once we reach the reference filter.
			if (curr_filter == ref_filter->m_Context)
				runflag = false;

			if (runflag && obs_source_enabled(curr_filter))
			{
				const auto flags = obs_source_get_output_flags(curr_filter);

				const bool is_asynchronous = test_bits<uint32_t>(flags, OBS_SOURCE_ASYNC_VIDEO);
				const bool is_same_type = ref_filter->m_Asynchronous == is_asynchronous;

				const bool is_vision_filter = s_Filters.count(curr_filter) != 0;
				const bool is_hybrid_render = is_vision_filter && s_Filters.at(curr_filter).get().m_HybridRender;

				if (is_same_type)
				{
					// Start chain if the previous filter is not a vision filter or we
					// are both synchronous, but the previous filter is hybrid render.
					start_chain = !is_vision_filter || (!is_asynchronous && is_hybrid_render);
				}
				else if (!is_same_type && is_asynchronous && is_hybrid_render)
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
		if (m_HybridRender && !m_Asynchronous)
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
				if (is_same_type)
				{
					// End chain if next filter is not another vision filter
					end_chain = !is_vision_filter;
					runflag = false;
				}
				else if (!is_same_type && is_asynchronous && is_hybrid_render)
				{
					// Always end the chain if we are synchronous and the 
					// next filter is an asynchronous hybrid render filter
					end_chain = true;
					runflag = false;
				}
			}

			// Activate search once we reach the reference filter
			if (curr_filter == ref_filter->m_Context)
				runflag = true;

		}, &search_state);

		return std::get<0>(search_state);
	}

//---------------------------------------------------------------------------------------------------------------------

	bool VisionFilter::acquire_render(FrameBuffer& buffer)
	{
		const auto target = obs_filter_get_target(m_Context);
		const uint32_t source_width = obs_source_get_base_width(target);
		const uint32_t source_height = obs_source_get_base_height(target);

		if (target == nullptr || source_width == 0 || source_height == 0)
			return false;

		// NOTE: sRGB is implicitly handled by the GS_RGBA format.
		// OpenGL can render directly to a GS_RGBA interop buffer,
		// but DirectX11 interop is supported with GS_RGBA_UNORM
		// only. So for DirectX11 to handle sRGB filters, we must
		// render to GS_RGBA then copy to the GS_RGBA_UNORM buffer.
		// The copy automatically handles sRGB conversions.

#ifdef _WIN32
		// DirectX11
		prepare_render_buffer(source_width, source_height);
		if (DefaultEffect::Acquire(m_Context, m_RenderBuffer))
		{
			prepare_interop_buffer(source_width, source_height);
			gs_copy_texture(m_InteropBuffer, m_RenderBuffer);

#else
		// OpenGL
		prepare_interop_buffer(source_width, source_height);
		if (DefaultEffect::Acquire(m_Context, m_InteropBuffer))
		{
#endif

			// Import the texture using interop and convert to YUV
			lvk::ocl::import_texture(m_InteropBuffer, m_ConversionBuffer);
			cv::cvtColor(m_ConversionBuffer, buffer.frame, cv::COLOR_RGBA2RGB);
			cv::cvtColor(buffer.frame, buffer.frame, cv::COLOR_RGB2YUV);

			return true;
		}
		return false;
	}

//---------------------------------------------------------------------------------------------------------------------

	gs_texture_t* VisionFilter::acquire_buffer(FrameBuffer& buffer)
	{
		prepare_interop_buffer(buffer.frame.cols, buffer.frame.rows);

		cv::cvtColor(buffer.frame, m_ConversionBuffer, cv::COLOR_YUV2RGB, 4);
		lvk::ocl::export_texture(m_ConversionBuffer, m_InteropBuffer);

		return m_InteropBuffer;
	}
	
//---------------------------------------------------------------------------------------------------------------------

	void VisionFilter::prepare_render_buffer(const uint32_t width, const uint32_t height)
	{
		LVK_ASSERT(gs_get_context() != nullptr);

		const bool outdated = m_RenderBuffer == nullptr
			|| gs_texture_get_width(m_RenderBuffer) != width
			|| gs_texture_get_height(m_RenderBuffer) != height;

		if (outdated)
		{
			gs_texture_destroy(m_RenderBuffer);
			m_RenderBuffer = gs_texture_create(
				width,
				height,
				GS_RGBA,
				1,
				nullptr,
				GS_RENDER_TARGET
			);
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void VisionFilter::prepare_interop_buffer(const uint32_t width, const uint32_t height)
	{
		LVK_ASSERT(gs_get_context() != nullptr);

		const bool outdated = m_InteropBuffer == nullptr
			|| gs_texture_get_width(m_InteropBuffer) != width
			|| gs_texture_get_height(m_InteropBuffer) != height;

		if (outdated)
		{
			gs_texture_destroy(m_InteropBuffer);

#ifdef _WIN32
			// DirectX11
			m_InteropBuffer = gs_texture_create(
				width,
				height,
				GS_RGBA_UNORM,
				1,
				nullptr,
				GS_SHARED_TEX
			);
#else
			// OpenGL (renders directly to buffer)
			m_InteropBuffer = gs_texture_create(
				width,
				height,
				GS_RGBA,
				1,
				nullptr,
				GS_SHARED_TEX | GS_RENDER_TARGET
			);
#endif
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void VisionFilter::filter(FrameBuffer& buffer)
	{
		filter(buffer.frame);
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
