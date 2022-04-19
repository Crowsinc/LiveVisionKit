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

#include "FrameBuffer.hpp"

#include "Diagnostics/Directives.hpp"
#include "FrameIngest.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	FrameBuffer::FrameBuffer()
		: frame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_FrameHandle(nullptr),
		  m_InteropTexture(nullptr),
		  m_InteropBuffer(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY)
	{}

//---------------------------------------------------------------------------------------------------------------------

	FrameBuffer::FrameBuffer(FrameBuffer&& buffer)
		: frame(buffer.frame),
		  m_FrameHandle(buffer.m_FrameHandle),
		  m_InteropTexture(buffer.m_InteropTexture),
		  m_InteropBuffer(buffer.m_InteropBuffer)
	{
		buffer.reset();
		buffer.frame.release();
		buffer.m_InteropBuffer.release();
		buffer.m_InteropTexture = nullptr;
	}

//---------------------------------------------------------------------------------------------------------------------

	FrameBuffer::~FrameBuffer()
	{
		if (m_InteropTexture != nullptr)
			gs_texture_destroy(m_InteropTexture);
	}

//---------------------------------------------------------------------------------------------------------------------

	void FrameBuffer::reset()
	{
		m_FrameHandle = nullptr;
	}

//---------------------------------------------------------------------------------------------------------------------

	void FrameBuffer::release(obs_source_t* owner)
	{
		LVK_ASSERT(owner != nullptr);

		obs_source_release_frame(owner, m_FrameHandle);
		m_FrameHandle = nullptr;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool FrameBuffer::empty() const
	{
		return m_FrameHandle == nullptr || frame.empty();
	}

//---------------------------------------------------------------------------------------------------------------------

	uint64_t FrameBuffer::timestamp() const
	{
		LVK_ASSERT(!empty());

		return m_FrameHandle->timestamp;
	}

//---------------------------------------------------------------------------------------------------------------------

	obs_source_frame* FrameBuffer::handle() const
	{
		return m_FrameHandle;
	}

//---------------------------------------------------------------------------------------------------------------------

	void FrameBuffer::upload(obs_source_frame* frame_handle)
	{
		LVK_ASSERT(frame_handle != nullptr);

		import_frame(frame_handle, frame);
		m_FrameHandle = frame_handle;
	}

//---------------------------------------------------------------------------------------------------------------------

	obs_source_frame* FrameBuffer::download() const
	{
		LVK_ASSERT(m_FrameHandle != nullptr);
		LVK_ASSERT(!frame.empty());

		//TODO: proper handling of errors when the frame is a different size than the frame handle specifies

		export_frame(frame, m_FrameHandle);
		return m_FrameHandle;
	}
	
//---------------------------------------------------------------------------------------------------------------------

	void FrameBuffer::prepare_interop_texture(const uint32_t width, const uint32_t height)
	{
		if (
			m_InteropTexture == nullptr 
			|| gs_texture_get_width(m_InteropTexture) != width
			|| gs_texture_get_height(m_InteropTexture) != height
		)
		{
			gs_texture_destroy(m_InteropTexture);
			m_InteropTexture = gs_texture_create(
				width,
				height,
				GS_RGBA_UNORM,
				1,
				nullptr,
				GS_RENDER_TARGET | GS_SHARED_TEX
			);
		}
	}
	
//---------------------------------------------------------------------------------------------------------------------

	bool FrameBuffer::acquire(const obs_source_t* source)
	{
		LVK_ASSERT(source != nullptr);
		LVK_ASSERT(ocl::supports_graphics_interop());

		const auto parent = obs_filter_get_target(source);
		const auto target = obs_filter_get_target(source);
		const uint32_t source_width = obs_source_get_base_width(target);
		const uint32_t source_height = obs_source_get_base_height(target);

		if (target == nullptr || parent == nullptr || source_width == 0 || source_height == 0)
		{
			LVK_WARN("Unable to acquire source texture");
			return false;
		}

		const auto target_flags = obs_source_get_output_flags(target);
		const bool allow_direct_render = (target_flags & OBS_SOURCE_CUSTOM_DRAW) == 0
									  && (target_flags & OBS_SOURCE_ASYNC) == 0;

		// Render the source to our interop texture
		// NOTE: Referenced from official gpu delay filter

		// Update render targets
		const auto prev_render_target = gs_get_render_target();
		const auto prev_z_stencil_target = gs_get_zstencil_target();

		prepare_interop_texture(source_width, source_height);
		gs_set_render_target(m_InteropTexture, nullptr);

		// Push new render state to stack
		gs_viewport_push();
		gs_projection_push();
		gs_matrix_push();
		gs_matrix_identity();
		gs_blend_state_push();
		gs_blend_function(GS_BLEND_ONE, GS_BLEND_ZERO);
		gs_set_viewport(0, 0, source_width, source_height);
		gs_ortho(0.0f, source_width, 0.0f, source_height, -100.0f, 100.0f);

		// Clear render texture and perform the render
		vec4 clear_color;
		vec4_zero(&clear_color);
		gs_clear(GS_CLEAR_COLOR, &clear_color, 0.0f, 0);

		if (target == parent && allow_direct_render)
			obs_source_default_render(target);
		else
			obs_source_video_render(target);

		// Restore render targets and state
		gs_matrix_pop();
		gs_projection_pop();
		gs_viewport_pop();
		gs_blend_state_pop();

		gs_set_render_target(
			prev_render_target,
			prev_z_stencil_target
		);
	
		// Import the texture using interop and convert to YUV
		ocl::import_texture(m_InteropTexture, m_InteropBuffer);
		cv::cvtColor(m_InteropBuffer, frame, cv::COLOR_RGBA2RGB);
		cv::cvtColor(frame, frame, cv::COLOR_RGB2YUV);

		return true;
	}
	
//---------------------------------------------------------------------------------------------------------------------

	void FrameBuffer::render()
	{
		LVK_ASSERT(!frame.empty());
		LVK_ASSERT(ocl::supports_graphics_interop());
		
		prepare_interop_texture(frame.cols, frame.rows);

		cv::cvtColor(frame, m_InteropBuffer, cv::COLOR_YUV2RGB, 4);
		ocl::export_texture(m_InteropBuffer, m_InteropTexture);

		// Render the interop texture as source output
		// NOTE: Referenced from official gpu delay filter

		const auto base_effect = obs_get_base_effect(OBS_EFFECT_DEFAULT);

		const bool use_srgb = gs_get_linear_srgb();
		const bool old_srgb_setting = gs_framebuffer_srgb_enabled();

		gs_enable_framebuffer_srgb(use_srgb);

		auto image_param = gs_effect_get_param_by_name(base_effect, "image");
		if (use_srgb)
			gs_effect_set_texture_srgb(image_param, m_InteropTexture);
		else
			gs_effect_set_texture(image_param, m_InteropTexture);

		while (gs_effect_loop(base_effect, "Draw"))
			gs_draw_sprite(m_InteropTexture, false, frame.cols, frame.rows);

		gs_enable_framebuffer_srgb(old_srgb_setting);
	}

//---------------------------------------------------------------------------------------------------------------------

	void FrameBuffer::operator=(FrameBuffer&& buffer)
	{
		frame = buffer.frame;
		m_FrameHandle = buffer.m_FrameHandle;
		m_InteropTexture = buffer.m_InteropTexture;
		m_InteropBuffer = buffer.m_InteropBuffer;

		buffer.reset();
		buffer.frame.release();
		buffer.m_InteropBuffer.release();
		buffer.m_InteropTexture = nullptr;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool FrameBuffer::operator==(obs_source_frame* frame_handle) const
	{
		return m_FrameHandle == frame_handle;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool FrameBuffer::operator==(const FrameBuffer& other) const
	{
		return m_FrameHandle == other.m_FrameHandle;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool FrameBuffer::operator!=(obs_source_frame* frame_handle) const
	{
		return m_FrameHandle != frame_handle;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool FrameBuffer::operator!=(const FrameBuffer& other) const
	{
		return m_FrameHandle != other.m_FrameHandle;
	}

//---------------------------------------------------------------------------------------------------------------------

}
