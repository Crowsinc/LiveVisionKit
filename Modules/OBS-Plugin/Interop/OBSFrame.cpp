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

#include "OBSFrame.hpp"

#include "InteropContext.hpp"
#include "Utility/Graphics.hpp"
#include "Utility/ScopedProfiler.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	OBSFrame::OBSFrame()
		: VideoFrame()
	{}

//---------------------------------------------------------------------------------------------------------------------

    OBSFrame::OBSFrame(const OBSFrame& frame)
        : VideoFrame(frame)
    {}

//---------------------------------------------------------------------------------------------------------------------

    OBSFrame::OBSFrame(OBSFrame&& frame) noexcept
        : VideoFrame(std::move(frame)),
          m_FrameIngest(std::move(frame.m_FrameIngest))
    {
        // TODO: add the rest of the move operations up here
    }

//---------------------------------------------------------------------------------------------------------------------

	OBSFrame::~OBSFrame()
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

		if(m_InteropTexture != nullptr)
		{
			gs_texture_destroy(m_InteropTexture);
            m_InteropTexture = nullptr;
		}

		obs_leave_graphics();
	}

//---------------------------------------------------------------------------------------------------------------------

    OBSFrame& OBSFrame::operator=(const OBSFrame& frame)
    {
        // TODO: implement
        return *this;
    }

//---------------------------------------------------------------------------------------------------------------------

    OBSFrame& OBSFrame::operator=(OBSFrame&& buffer) noexcept
	{
        // TODO: implement properly
		Frame::operator=(std::move(buffer));
	    return *this;
    }

//---------------------------------------------------------------------------------------------------------------------

    bool OBSFrame::to_obs_frame(obs_source_frame* obs_frame) const
    {
        LVK_ASSERT(obs_frame != nullptr);

        if(m_FrameIngest == nullptr || m_FrameIngest->obs_format() != obs_frame->format)
        {
            // Select the correct frame ingest for the frame.
            if(m_FrameIngest = FrameIngest::Select(obs_frame->format); !m_FrameIngest)
                return false;
        }

        m_FrameIngest->download_ocl_frame(*this, obs_frame);
        return true;
    }

//---------------------------------------------------------------------------------------------------------------------

	bool OBSFrame::from_obs_frame(const obs_source_frame* obs_frame)
	{
		LVK_ASSERT(obs_frame != nullptr);

        if(m_FrameIngest == nullptr || m_FrameIngest->obs_format() != obs_frame->format)
        {
            // Select the correct frame ingest for the frame.
            if(m_FrameIngest = FrameIngest::Select(obs_frame->format); !m_FrameIngest)
                return false;
        }

        m_FrameIngest->upload_obs_frame(obs_frame, *this);
        return true;
	}

//---------------------------------------------------------------------------------------------------------------------


    void OBSFrame::to_obs_texture(gs_texture* texture) const
    {
        LVK_ASSERT(texture != nullptr);
        LVK_ASSERT(gs_texture_get_color_format(texture) == GS_RGBA);
        LVK_ASSERT(gs_texture_get_height(texture) == rows);
        LVK_ASSERT(gs_texture_get_width(texture) == cols);
        LVK_PROFILE;

        // Ensure the frame is in RGBA format
        // TODO: add direct RGBA format to the VideoFrame API.
        this->viewAsFormat(m_FormatBuffer, VideoFrame::RGB);
        cv::cvtColor(m_FormatBuffer, m_InteropBuffer, cv::COLOR_RGB2RGBA);

        if(lvk::ocl::InteropContext::Available())
        {
            prepare_interop_texture(cols, rows);

            lvk::ocl::InteropContext::Export(m_InteropBuffer, m_InteropTexture);
            gs_copy_texture(texture, m_InteropTexture);
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

            m_InteropBuffer.copyTo(cv::Mat(
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

	void OBSFrame::from_obs_texture(gs_texture* texture)
	{
		LVK_ASSERT(texture != nullptr);
		LVK_ASSERT(gs_texture_get_color_format(texture) == GS_RGBA);
        LVK_PROFILE;

		const uint32_t texture_width = gs_texture_get_width(texture);
		const uint32_t texture_height = gs_texture_get_height(texture);

		if(lvk::ocl::InteropContext::Available())
		{
			prepare_interop_texture(texture_width, texture_height);

			// NOTE: sRGB is implicitly handled by the GS_RGBA format.
			// OpenGL supports interop through GS_RGBA, but DirectX11
			// does not. So for DirectX11 to handle sRGB filters, we must
			// render to GS_RGBA then copy to the GS_RGBA_UNORM buffer.
			// The copy automatically handles sRGB conversions.

			gs_copy_texture(m_InteropTexture, texture);
			
			lvk::ocl::InteropContext::Import(m_InteropTexture, m_InteropBuffer);
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
			).copyTo(m_InteropBuffer);
			
			gs_stagesurface_unmap(m_ReadBuffer);
		}

		// Convert from RGBA to RGB
		cv::cvtColor(m_InteropBuffer, *this, cv::COLOR_RGBA2RGB);
        format = VideoFrame::RGB;
	}

//---------------------------------------------------------------------------------------------------------------------

	void OBSFrame::prepare_interop_texture(const uint32_t width, const uint32_t height) const
	{
		prepare_texture(
			m_InteropTexture,
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
