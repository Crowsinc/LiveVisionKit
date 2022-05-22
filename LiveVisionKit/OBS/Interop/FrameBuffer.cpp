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
#include "OBS/Effects/DefaultEffect.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	FrameBuffer::FrameBuffer()
		: timestamp(0)
	{}
	
//---------------------------------------------------------------------------------------------------------------------

	FrameBuffer::FrameBuffer(FrameBuffer&& buffer)
		: frame(buffer.frame),
		  timestamp(buffer.timestamp)
	{
		buffer.frame.release();
		buffer.timestamp = 0;
	}

//---------------------------------------------------------------------------------------------------------------------

	void FrameBuffer::operator=(FrameBuffer&& buffer)
	{
		frame = buffer.frame;
		timestamp = buffer.timestamp;

		buffer.frame.release();
		buffer.timestamp = 0;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool FrameBuffer::empty() const
	{
		return frame.empty();
	}
	
//---------------------------------------------------------------------------------------------------------------------

	void FrameBuffer::load(obs_source_frame* obs_frame)
	{
		LVK_ASSERT(obs_frame != nullptr);

		import_frame(obs_frame, frame);
		timestamp = obs_frame->timestamp;
	}

//---------------------------------------------------------------------------------------------------------------------

	void FrameBuffer::save(obs_source_frame* obs_frame)
	{
		LVK_ASSERT(obs_frame != nullptr);

		export_frame(frame, obs_frame);
	}

//---------------------------------------------------------------------------------------------------------------------

}
