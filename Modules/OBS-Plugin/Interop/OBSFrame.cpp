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
#include "Utility/ScopedProfiler.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	OBSFrame::OBSFrame()
		: VideoFrame()
	{}

//---------------------------------------------------------------------------------------------------------------------

    // Only copy the frame data over.
    OBSFrame::OBSFrame(const OBSFrame& frame)
        : VideoFrame(frame)
    {}

//---------------------------------------------------------------------------------------------------------------------

    OBSFrame::OBSFrame(OBSFrame&& frame) noexcept
        : VideoFrame(std::move(frame)),
          m_FrameIngest(std::move(frame.m_FrameIngest)),
          m_WriteBuffer(std::move(frame.m_WriteBuffer)),
          m_ReadBuffer(std::move(frame.m_ReadBuffer)),
          m_ExportTexture(frame.m_ExportTexture)
    {
        frame.m_ExportTexture = nullptr;
    }

//---------------------------------------------------------------------------------------------------------------------

	OBSFrame::~OBSFrame()
	{
		obs_enter_graphics();

		if(m_ExportTexture != nullptr)
		{
			gs_texture_destroy(m_ExportTexture);
            m_ExportTexture = nullptr;
		}

		obs_leave_graphics();
	}

//---------------------------------------------------------------------------------------------------------------------

    OBSFrame& OBSFrame::operator=(const OBSFrame& frame)
    {
        // Only copy the frame data over.
        Frame::operator=(frame);

        return *this;
    }

//---------------------------------------------------------------------------------------------------------------------

    OBSFrame& OBSFrame::operator=(OBSFrame&& buffer) noexcept
	{
        m_FrameIngest = std::move(buffer.m_FrameIngest);
        m_WriteBuffer = std::move(buffer.m_WriteBuffer);
        m_ReadBuffer = std::move(buffer.m_ReadBuffer);
        m_ExportTexture = buffer.m_ExportTexture;
        buffer.m_ExportTexture = nullptr;

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

        if(lvk::ocl::InteropContext::Available())
        {
            prepare_interop_texture(cols, rows);

            // NOTE: We perform a copy here to handle sRGB textures on DirectX11.
            viewAsFormat(m_ExportBuffer, VideoFrame::RGBA);
            lvk::ocl::InteropContext::Export(m_ExportBuffer, m_ExportTexture);
            gs_copy_texture(texture, m_ExportTexture);
        }
        else
        {
            prepare_texture(
                m_ExportTexture,
                cols,rows,
                GS_RGBA,
                GS_DYNAMIC
            );

            uint8_t* mapped_ptr = m_WriteBuffer.map(m_ExportTexture);

            if(m_WriteBuffer.requires_rgba())
            {
                // Upload as RGBA
                viewAsFormat(m_ExportBuffer, VideoFrame::RGBA);
                m_ExportBuffer.copyTo(cv::Mat(
                    rows,
                    cols,
                    CV_8UC4,
                    mapped_ptr
                ));
            }
            else
            {
                // Upload as RGB
                viewAsFormat(m_ExportBuffer, VideoFrame::RGB);
                m_ExportBuffer.copyTo(cv::Mat(
                    rows,
                    cols,
                    CV_8UC3,
                    mapped_ptr
                ));
            }

            m_WriteBuffer.flush();

            gs_copy_texture(texture, m_ExportTexture);
        }
    }

//---------------------------------------------------------------------------------------------------------------------

	void OBSFrame::from_obs_texture(gs_texture* texture)
	{
		LVK_ASSERT(texture != nullptr);
		LVK_ASSERT(gs_texture_get_color_format(texture) == GS_RGBA);
        LVK_PROFILE;

		const int texture_width = static_cast<int>(gs_texture_get_width(texture));
		const int texture_height = static_cast<int>(gs_texture_get_height(texture));

		if(lvk::ocl::InteropContext::Available())
		{
			prepare_interop_texture(texture_width, texture_height);

            // NOTE: We perform a copy here to handle sRGB textures on DirectX11.
			gs_copy_texture(m_ExportTexture, texture);
			lvk::ocl::InteropContext::Import(m_ExportTexture, *this);

            format = VideoFrame::RGBA;
		}
		else
		{
            uint8_t* mapped_ptr = m_ReadBuffer.map(texture);

            if(m_ReadBuffer.requires_rgba())
            {
                // Copy as RGBA
                cv::Mat(
                    texture_height, texture_width,
                    CV_8UC4,
                    mapped_ptr
                ).copyTo(*this);
                format = VideoFrame::RGBA;
            }
            else
            {
                // Copy as RGB
                cv::Mat(
                    texture_height, texture_width,
                    CV_8UC3,
                    mapped_ptr
                ).copyTo(*this);
                format = VideoFrame::RGB;
            }

            m_ReadBuffer.flush();
		}

        // Ensure output is in RGB.
        reformat(VideoFrame::RGB);
	}

//---------------------------------------------------------------------------------------------------------------------

	void OBSFrame::prepare_interop_texture(const int width, const int height) const
	{
		prepare_texture(
                m_ExportTexture,
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
