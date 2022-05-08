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
#include "DefaultEffect.hpp"

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

		prepare_interop_texture(source_width, source_height);
		if (DefaultEffect::Acquire(source, m_InteropTexture))
		{
			// Import the texture using interop and convert to YUV
			ocl::import_texture(m_InteropTexture, m_InteropBuffer);
			cv::cvtColor(m_InteropBuffer, frame, cv::COLOR_RGBA2RGB);
			cv::cvtColor(frame, frame, cv::COLOR_RGB2YUV);

			return true;
		}
		return false;
	}
	
//---------------------------------------------------------------------------------------------------------------------

	void FrameBuffer::render()
	{
		LVK_ASSERT(!frame.empty());
		LVK_ASSERT(ocl::supports_graphics_interop());
		
		prepare_interop_texture(frame.cols, frame.rows);

		cv::cvtColor(frame, m_InteropBuffer, cv::COLOR_YUV2RGB, 4);
		ocl::export_texture(m_InteropBuffer, m_InteropTexture);

		DefaultEffect::Render(m_InteropTexture, frame.size());
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
