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

#pragma once

#include <obs-module.h>

// Helper Functions
namespace lvk
{

	void prepare_texture(
		gs_texture_t*& texture,
		const uint32_t width,
		const uint32_t height,
		const gs_color_format format = GS_RGBA,
		const uint32_t flags = 0
	);

	void prepare_staging_surface(
		gs_stagesurf_t*& surface,
		const uint32_t width,
		const uint32_t height,
		const gs_color_format format = GS_RGBA
	);

}

// Transparent Classes
namespace lvk
{
    // TODO: Make these a less of a hack.

    struct RGBTextureWriteBuffer
    {
        RGBTextureWriteBuffer() = default;

        RGBTextureWriteBuffer(RGBTextureWriteBuffer&& other) noexcept;

        RGBTextureWriteBuffer(const RGBTextureWriteBuffer& other) = delete;

        ~RGBTextureWriteBuffer();


        RGBTextureWriteBuffer& operator=(RGBTextureWriteBuffer&& other) noexcept;


        uint8_t* map(gs_texture_t* target);

        void flush();

        static bool requires_rgba();

    private:

        static bool use_custom_buffers();

        uint64_t m_BufferSize = 0;
        uint32_t m_BufferObject = 0;
        uint8_t* m_MappedData = nullptr;
        gs_texture_t* m_Target = nullptr;

        bool m_SafeMode = false;
    };

    struct RGBTextureReadBuffer
    {
        RGBTextureReadBuffer() = default;

        RGBTextureReadBuffer(RGBTextureReadBuffer&& other) noexcept;

        RGBTextureReadBuffer(const RGBTextureReadBuffer& other) = delete;

        ~RGBTextureReadBuffer();


        RGBTextureReadBuffer& operator=(RGBTextureReadBuffer&& other) noexcept;


        uint8_t* map(gs_texture_t* target);

        void flush();

        static bool requires_rgba();

    private:

        static bool use_custom_buffers();

        uint64_t m_BufferSize = 0;
        uint32_t m_BufferObject = 0;
        uint8_t* m_MappedData = nullptr;
        gs_stagesurf_t* m_StagingSurface = nullptr;
    };

}



