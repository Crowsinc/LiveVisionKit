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

#include "Diagnostics/Assert.hpp"
#include "FrameIngest.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	FrameBuffer::FrameBuffer()
		: frame(cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY),
		  m_SourceFrame(nullptr)
	{}

//---------------------------------------------------------------------------------------------------------------------

	FrameBuffer::FrameBuffer(FrameBuffer&& buffer)
		: frame(buffer.frame),
		  m_SourceFrame(buffer.m_SourceFrame)
	{
		buffer.frame.release();
		buffer.m_SourceFrame = nullptr;
	}

//---------------------------------------------------------------------------------------------------------------------

	void FrameBuffer::reset()
	{
		m_SourceFrame = nullptr;
	}

//---------------------------------------------------------------------------------------------------------------------

	void FrameBuffer::release(obs_source_t* owner)
	{
		LVK_ASSERT(owner != nullptr);

		obs_source_release_frame(owner, m_SourceFrame);
		m_SourceFrame = nullptr;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool FrameBuffer::empty() const
	{
		return m_SourceFrame == nullptr;
	}

//---------------------------------------------------------------------------------------------------------------------

	void FrameBuffer::upload(obs_source_frame* source_frame)
	{
		LVK_ASSERT(source_frame != nullptr);

		frame << source_frame;
		m_SourceFrame = source_frame;
	}

//---------------------------------------------------------------------------------------------------------------------

	obs_source_frame* FrameBuffer::download() const
	{
		LVK_ASSERT(m_SourceFrame != nullptr && !frame.empty());

		frame >> m_SourceFrame;
		return m_SourceFrame;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool FrameBuffer::operator==(obs_source_frame* source_frame) const
	{
		return m_SourceFrame == source_frame;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool FrameBuffer::operator==(const FrameBuffer& other) const
	{
		return m_SourceFrame == other.m_SourceFrame;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool FrameBuffer::operator!=(obs_source_frame* source_frame) const
	{
		return m_SourceFrame != source_frame;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool FrameBuffer::operator!=(const FrameBuffer& other) const
	{
		return m_SourceFrame != other.m_SourceFrame;
	}

//---------------------------------------------------------------------------------------------------------------------

}
