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

#include "Graphics.hpp"

#include <LiveVisionKit.hpp>
#include "3rdParty/glad.h"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	void prepare_texture(
		gs_texture_t*& texture,
		const uint32_t width,
		const uint32_t height,
		const gs_color_format format,
		const uint32_t flags
	)
	{
		LVK_ASSERT(gs_get_context() != nullptr);

		const bool outdated = texture == nullptr
			|| gs_texture_get_width(texture) != width
			|| gs_texture_get_height(texture) != height
			|| gs_texture_get_color_format(texture) != format;

		if(outdated)
		{
			gs_texture_destroy(texture);
			texture = gs_texture_create(
				width,
				height,
				format,
				1,
				nullptr,
				flags
			);
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void prepare_staging_surface(
		gs_stagesurf_t*& surface,
		const uint32_t width,
		const uint32_t height,
		const gs_color_format format
	)
	{
		LVK_ASSERT(gs_get_context() != nullptr);

		const bool outdated = surface == nullptr
			|| gs_stagesurface_get_width(surface) != width
			|| gs_stagesurface_get_height(surface) != height
			|| gs_stagesurface_get_color_format(surface) != format;

		if(outdated)
		{
			gs_stagesurface_destroy(surface);
			surface = gs_stagesurface_create(
				width,
				height,
				format
			);
		}
	}

//---------------------------------------------------------------------------------------------------------------------

// TODO: handle this properly
#ifdef FAST_GL_DMA
	bool init_glad() {thread_local auto status = gladLoadGL(); return status != 0;}
#endif

//---------------------------------------------------------------------------------------------------------------------
//  Texture Write Buffer
//---------------------------------------------------------------------------------------------------------------------

	RGBTextureWriteBuffer::RGBTextureWriteBuffer(RGBTextureWriteBuffer&& other) noexcept
		: m_Target(other.m_Target),
		  m_BufferSize(other.m_BufferSize),
		  m_MappedData(other.m_MappedData),
		  m_BufferObject(other.m_BufferObject)
	{
		other.m_BufferSize = 0;
		other.m_BufferObject = 0;
		other.m_MappedData = nullptr;
		other.m_Target = nullptr;
	}

//---------------------------------------------------------------------------------------------------------------------

	RGBTextureWriteBuffer::~RGBTextureWriteBuffer()
	{
		obs_enter_graphics();

#ifdef FAST_GL_DMA
		if(m_BufferObject != 0)
		{
			glDeleteBuffers(1, &m_BufferObject);
		}
#endif

		obs_leave_graphics();
	}

//---------------------------------------------------------------------------------------------------------------------

	RGBTextureWriteBuffer& RGBTextureWriteBuffer::operator=(RGBTextureWriteBuffer&& other) noexcept
	{
		m_Target = other.m_Target;
		m_BufferSize = other.m_BufferSize;
		m_MappedData = other.m_MappedData;
		m_BufferObject = other.m_BufferObject;

		other.m_Target = nullptr;
		other.m_BufferSize = 0;
		other.m_BufferObject = 0;
		other.m_MappedData = nullptr;

		return *this;
	}

//---------------------------------------------------------------------------------------------------------------------

	uint8_t* RGBTextureWriteBuffer::map(gs_texture_t* target)
	{
		LVK_ASSERT(target != nullptr);

		m_Target = target;
		const uint64_t width = gs_texture_get_width(target);
		const uint64_t height = gs_texture_get_height(target);

#ifdef FAST_GL_DMA
		// If we are on Linux with access to ARB buffer storage,
		// then use persistent buffer mapping to speed things up.
		if(use_custom_buffers())
		{
			// Create a new buffer if the existing one isn't the right size.
			const uint64_t rgb_buffer_size = width * height * 3u;
			if(m_BufferSize != rgb_buffer_size)
			{
				if(m_BufferObject != 0) glDeleteBuffers(1, &m_BufferObject);

				glGenBuffers(1, &m_BufferObject);
				glBindBuffer(GL_PIXEL_UNPACK_BUFFER, m_BufferObject);
				m_BufferSize = rgb_buffer_size;

				// Create the PBO.
				glBufferStorage(
					GL_PIXEL_UNPACK_BUFFER,
					static_cast<GLsizeiptr>(m_BufferSize),
					nullptr,
					GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT
				);

				// Map the PBO using persistent mapping.
				m_MappedData = (uint8_t*)glMapBufferRange(
					GL_PIXEL_UNPACK_BUFFER,
					0, static_cast<GLsizeiptr>(m_BufferSize),
					GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT | GL_MAP_FLUSH_EXPLICIT_BIT
				);
			}
		}
        else
#endif
		{
			uint32_t linesize = 0;
			gs_texture_map(target, &m_MappedData, &linesize);
		}

		return m_MappedData;
	}

//---------------------------------------------------------------------------------------------------------------------

	void RGBTextureWriteBuffer::flush()
	{
		LVK_ASSERT(m_Target != nullptr);

#ifdef FAST_GL_DMA
		if(use_custom_buffers())
		{
			LVK_ASSERT(m_BufferObject != 0);

			glBindBuffer(GL_PIXEL_UNPACK_BUFFER, m_BufferObject);
			glBindTexture(GL_TEXTURE_2D, *(GLuint*)gs_texture_get_obj(m_Target));

			glFlushMappedBufferRange(
				GL_PIXEL_UNPACK_BUFFER,
				0, static_cast<GLsizeiptr>(m_BufferSize)
			);

			// NOTE: the underlying texture is RGBA, but we send it in RGB.
			glTexImage2D(
				GL_TEXTURE_2D,
				0,
				GL_RGBA,
				static_cast<GLsizei>(gs_texture_get_width(m_Target)),
				static_cast<GLsizei>(gs_texture_get_height(m_Target)),
				0,
				GL_RGB,
				GL_UNSIGNED_BYTE,
				(void*)0
			 );

			glBindTexture(GL_TEXTURE_2D, 0);
			glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
		}
		else
#endif
		{
			gs_texture_unmap(m_Target);
		}

		m_Target = nullptr;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool RGBTextureWriteBuffer::requires_rgba()
	{
		return !use_custom_buffers();
	}

//---------------------------------------------------------------------------------------------------------------------

	bool RGBTextureWriteBuffer::use_custom_buffers()
	{
#ifdef FAST_GL_DMA
		return init_glad() && GLAD_GL_ARB_buffer_storage;
#else
		return false;
#endif
	}

//---------------------------------------------------------------------------------------------------------------------
//  Texture Read Buffer
//---------------------------------------------------------------------------------------------------------------------

	RGBTextureReadBuffer::RGBTextureReadBuffer(RGBTextureReadBuffer&& other) noexcept
		: m_BufferSize(other.m_BufferSize),
		  m_MappedData(other.m_MappedData),
		  m_BufferObject(other.m_BufferObject),
		  m_StagingSurface(other.m_StagingSurface)
	{
		other.m_BufferSize = 0;
		other.m_BufferObject = 0;
		other.m_MappedData = nullptr;
		other.m_StagingSurface = nullptr;
	}

//---------------------------------------------------------------------------------------------------------------------

	RGBTextureReadBuffer::~RGBTextureReadBuffer()
	{
		obs_enter_graphics();

#ifdef FAST_GL_DMA
		if(m_BufferObject != 0)
		{
			glDeleteBuffers(1, &m_BufferObject);
		}
#endif
		if(m_StagingSurface != nullptr)
		{
			gs_stagesurface_destroy(m_StagingSurface);
		}

		obs_leave_graphics();
	}

//---------------------------------------------------------------------------------------------------------------------

	RGBTextureReadBuffer& RGBTextureReadBuffer::operator=(RGBTextureReadBuffer&& other) noexcept
	{
		m_BufferSize = other.m_BufferSize;
		m_MappedData = other.m_MappedData;
		m_BufferObject = other.m_BufferObject;
		m_StagingSurface = other.m_StagingSurface;

		other.m_BufferSize = 0;
		other.m_BufferObject = 0;
		other.m_MappedData = nullptr;
		other.m_StagingSurface = nullptr;

		return *this;
	}

//---------------------------------------------------------------------------------------------------------------------

	uint8_t* RGBTextureReadBuffer::map(gs_texture_t* target)
	{
		LVK_ASSERT(target != nullptr);

		const uint64_t width = gs_texture_get_width(target);
		const uint64_t height = gs_texture_get_height(target);

#ifdef FAST_GL_DMA
		// If we are on Linux with access to ARB buffer storage,
		// then use persistent buffer mapping to speed things up.
		if(use_custom_buffers())
		{
			// Create a new buffer if the existing one isn't the right size.
			const uint64_t rgb_buffer_size = width * height * 3u;
			if(m_BufferSize != rgb_buffer_size)
			{
				if(m_BufferObject != 0) glDeleteBuffers(1, &m_BufferObject);

				glGenBuffers(1, &m_BufferObject);
				glBindBuffer(GL_PIXEL_PACK_BUFFER, m_BufferObject);
				m_BufferSize = rgb_buffer_size;

				// Create the PBO.
				glBufferStorage(
					GL_PIXEL_PACK_BUFFER,
					static_cast<GLsizeiptr>(m_BufferSize),
					nullptr,
					GL_MAP_READ_BIT | GL_MAP_PERSISTENT_BIT | GL_MAP_COHERENT_BIT
				);

				// Map the PBO using persistent mapping.
				m_MappedData = (uint8_t*)glMapBufferRange(
					GL_PIXEL_PACK_BUFFER,
					0, static_cast<GLsizeiptr>(m_BufferSize),
					GL_MAP_READ_BIT | GL_MAP_PERSISTENT_BIT | GL_MAP_COHERENT_BIT
				);
			}

			glBindBuffer(GL_PIXEL_PACK_BUFFER, m_BufferObject);
			glBindTexture(GL_TEXTURE_2D, *(GLuint*)gs_texture_get_obj(target));

			// Copy the texture onto the PBO
			// NOTE: the underlying texture is RGBA, but we grab it in RGB.
			glGetTexImage(
				GL_TEXTURE_2D,
				0,
				GL_RGB,
				GL_UNSIGNED_BYTE,
				(void*)0
			);

            // TODO: this isn't very efficient, but we
            // need a synchronization point somewhere here.
            glFinish();

			glBindTexture(GL_TEXTURE_2D, 0);
			glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
		}
        else
#endif
        {
			prepare_staging_surface(
				m_StagingSurface,
				width, height,
				GS_RGBA
			);

			uint32_t linesize = 0;
			gs_stage_texture(m_StagingSurface, target);
			gs_stagesurface_map(m_StagingSurface, &m_MappedData, &linesize);
		}

		return m_MappedData;
	}

//---------------------------------------------------------------------------------------------------------------------

	void RGBTextureReadBuffer::flush()
	{
#ifdef FAST_GL_DMA
		if(use_custom_buffers())
		{
            // Do nothing..
		}
		else
#endif
		{
			gs_stagesurface_unmap(m_StagingSurface);
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	bool RGBTextureReadBuffer::requires_rgba()
	{
		return !use_custom_buffers();
	}

//---------------------------------------------------------------------------------------------------------------------

	bool RGBTextureReadBuffer::use_custom_buffers()
	{
#ifdef FAST_GL_DMA
		return init_glad() && GLAD_GL_ARB_buffer_storage;
#else
		return false;
#endif
	}

//---------------------------------------------------------------------------------------------------------------------

}