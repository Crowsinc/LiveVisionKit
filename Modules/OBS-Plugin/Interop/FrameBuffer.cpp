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

#include "InteropContext.hpp"
#include "Utility/Graphics.hpp"
#include "Utility/ScopedProfiler.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	FrameBuffer::FrameBuffer()
		: Frame()
	{}

//---------------------------------------------------------------------------------------------------------------------

	FrameBuffer::~FrameBuffer()
	{
		obs_enter_graphics();

		if(m_ReadBuffer != nullptr)
		{
			gs_stagesurface_destroy(m_ReadBuffer);
			m_ReadBuffer = nullptr;
		}

		if(m_WriteBuffer != nullptr)
		{
			gs_texture_destroy(m_WriteBuffer);
			m_WriteBuffer = nullptr;
		}

		if(m_InteropBuffer != nullptr)
		{
			gs_texture_destroy(m_InteropBuffer);
			m_InteropBuffer = nullptr;
		}

		obs_leave_graphics();
	}

//---------------------------------------------------------------------------------------------------------------------

	FrameBuffer::FrameBuffer(FrameBuffer&& buffer) noexcept
		: Frame(static_cast<Frame&&>(buffer))
	{}

//---------------------------------------------------------------------------------------------------------------------

    FrameBuffer& FrameBuffer::operator=(FrameBuffer&& buffer) noexcept
	{
		Frame::operator=(std::move(buffer));
	    return *this;
    }

//---------------------------------------------------------------------------------------------------------------------

	bool FrameBuffer::try_upload_frame(obs_source_frame* obs_frame)
	{
		LVK_ASSERT(obs_frame != nullptr);

		if(!m_FrameIngest || m_FrameIngest->format() != obs_frame->format)
			m_FrameIngest = FrameIngest::Select(obs_frame->format);

		if(m_FrameIngest)
		{
			m_FrameIngest->upload(obs_frame, *this);
			timestamp = obs_frame->timestamp;
			return true;
		}
		else return false;

	}

//---------------------------------------------------------------------------------------------------------------------

	bool FrameBuffer::try_download_frame(obs_source_frame* obs_frame)
	{
		LVK_ASSERT(obs_frame != nullptr);

		if(!m_FrameIngest || m_FrameIngest->format() != obs_frame->format)
			m_FrameIngest = FrameIngest::Select(obs_frame->format);

		if(m_FrameIngest)
		{
			m_FrameIngest->download(*this, obs_frame);
			timestamp = obs_frame->timestamp;
			return true;
		}
		else return false;
	}

//---------------------------------------------------------------------------------------------------------------------

	void FrameBuffer::import_texture(gs_texture* texture)
	{
		LVK_ASSERT(texture != nullptr);
		LVK_ASSERT(gs_texture_get_color_format(texture) == GS_RGBA);
        LVK_PROFILE;

		const uint32_t texture_width = gs_texture_get_width(texture);
		const uint32_t texture_height = gs_texture_get_height(texture);

		if(lvk::ocl::InteropContext::Available())
		{
			prepare_interop_buffer(texture_width, texture_height);

			// NOTE: sRGB is implicitly handled by the GS_RGBA format.
			// OpenGL supports interop through GS_RGBA, but DirectX11
			// does not. So for DirectX11 to handle sRGB filters, we must
			// render to GS_RGBA then copy to the GS_RGBA_UNORM buffer.
			// The copy automatically handles sRGB conversions.

			gs_copy_texture(m_InteropBuffer, texture);
			
			lvk::ocl::InteropContext::Import(m_InteropBuffer, m_ConversionBuffer);
		}
		else
		{
			prepare_staging_surface(
				m_ReadBuffer,
				texture_width,
				texture_height,
				GS_RGBA
			);

			uint32_t line_size = 0;
			gs_stage_texture(m_ReadBuffer, texture);
			gs_stagesurface_map(m_ReadBuffer, &m_MappedData, &line_size);
			
			cv::Mat(
                static_cast<int>(texture_height),
                static_cast<int>(texture_width),
                CV_8UC4,
                m_MappedData,
                line_size
			).copyTo(m_ConversionBuffer);
			
			gs_stagesurface_unmap(m_ReadBuffer);
		}

		// Convert from RGBA to YUV
		cv::cvtColor(m_ConversionBuffer, *this, cv::COLOR_RGBA2RGB);
		cv::cvtColor(*this, *this, cv::COLOR_RGB2YUV);
	}

//---------------------------------------------------------------------------------------------------------------------

	void FrameBuffer::export_texture(gs_texture* texture)
	{
		LVK_ASSERT(texture != nullptr);
		LVK_ASSERT(gs_texture_get_color_format(texture) == GS_RGBA);
		LVK_ASSERT(gs_texture_get_width(texture) == cols);
		LVK_ASSERT(gs_texture_get_height(texture) == rows);
        LVK_PROFILE;

		// Convert from YUV to RGBA
		cv::cvtColor(*this, m_ConversionBuffer, cv::COLOR_YUV2RGB, 4);

		if(lvk::ocl::InteropContext::Available())
		{
			prepare_interop_buffer(cols, rows);
			
			lvk::ocl::InteropContext::Export(m_ConversionBuffer, m_InteropBuffer);
			gs_copy_texture(texture, m_InteropBuffer);
		}
		else
		{
			prepare_texture(
				m_WriteBuffer,
				cols,
				rows,
				GS_RGBA,
				GS_DYNAMIC
			);

			uint32_t linesize = 0;
			gs_texture_map(m_WriteBuffer, &m_MappedData, &linesize);
		
			m_ConversionBuffer.copyTo(cv::Mat(
				rows,
				cols,
				CV_8UC4,
				m_MappedData,
				linesize
			));

			gs_texture_unmap(m_WriteBuffer);

			gs_copy_texture(texture, m_WriteBuffer);
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void FrameBuffer::prepare_interop_buffer(const uint32_t width, const uint32_t height)
	{
		prepare_texture(
			m_InteropBuffer,
			width,
			height,
#ifdef _WIN32 
			GS_RGBA_UNORM, // DirectX
#else
			GS_RGBA, // OpenGL
#endif
			GS_SHARED_TEX | GS_RENDER_TARGET
		);
	}

//---------------------------------------------------------------------------------------------------------------------

}
