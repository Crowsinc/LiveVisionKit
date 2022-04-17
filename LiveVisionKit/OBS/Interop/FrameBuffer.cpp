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
		  m_FrameHandle(nullptr)
	{}

//---------------------------------------------------------------------------------------------------------------------

	FrameBuffer::FrameBuffer(FrameBuffer&& buffer)
		: frame(buffer.frame),
		  m_FrameHandle(buffer.m_FrameHandle)
	{
		buffer.reset();
		buffer.frame.release();
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
		return m_FrameHandle == nullptr;
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

	void FrameBuffer::acquire(obs_source_t* source)
	{
		LVK_ASSERT(source != nullptr);
	
		lvk::acquire(source, frame);
	}
	
//---------------------------------------------------------------------------------------------------------------------

	void FrameBuffer::render(obs_source_t* source)
	{
		LVK_ASSERT(source != nullptr);
		LVK_ASSERT(!frame.empty());
	
		lvk::render(source, frame);
	}

//---------------------------------------------------------------------------------------------------------------------

	void FrameBuffer::copy_to(gs_texture_t* texture)
	{
		LVK_ASSERT(texture != nullptr);
		LVK_ASSERT(!frame.empty());
		
		lvk::export_texture(frame, texture);
	}

//---------------------------------------------------------------------------------------------------------------------

	void FrameBuffer::operator=(FrameBuffer&& buffer)
	{
		frame = buffer.frame;
		m_FrameHandle = buffer.m_FrameHandle;

		buffer.reset();
		buffer.frame.release();
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
