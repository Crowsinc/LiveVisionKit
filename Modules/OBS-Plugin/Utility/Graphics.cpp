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

		if (outdated)
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

		if (outdated)
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

}