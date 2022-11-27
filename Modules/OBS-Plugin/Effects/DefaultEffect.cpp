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

#include "DefaultEffect.hpp"

#include <LiveVisionKit.hpp>

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	bool DefaultEffect::Acquire(
		const obs_source_t* source,
		gs_texture_t* texture
	)
	{
		LVK_ASSERT(source != nullptr);
		LVK_ASSERT(texture != nullptr);

		const auto parent = obs_filter_get_parent(source);
		const auto target = obs_filter_get_target(source);
		
		const cv::Size source_size(
			static_cast<int>(obs_source_get_base_width(target)),
			static_cast<int>(obs_source_get_base_height(target))
		);
		const cv::Size texture_size(
			static_cast<int>(gs_texture_get_width(texture)),
			static_cast<int>(gs_texture_get_height(texture))
		);

		const bool render_valid = target != nullptr && parent != nullptr
							   && source_size.width > 0 && source_size.height > 0
							   && texture_size == source_size;

		if (render_valid)
		{
			const auto target_flags = obs_source_get_output_flags(target);
			const bool allow_direct_render = !test_bits<uint32_t>(target_flags, OBS_SOURCE_CUSTOM_DRAW)
									      && !test_bits<uint32_t>(target_flags, OBS_SOURCE_ASYNC);

			// Update render targets
			const auto prev_render_target = gs_get_render_target();
			const auto prev_z_stencil_target = gs_get_zstencil_target();
			
			gs_set_render_target(texture, nullptr);
			
			// Push new render state to stack
			gs_viewport_push();
			gs_projection_push();
			gs_matrix_push();
			gs_matrix_identity();
			gs_blend_state_push();
			gs_blend_function(GS_BLEND_ONE, GS_BLEND_ZERO);
			gs_set_viewport(0, 0, source_size.width, source_size.height);
			gs_ortho(
                0.0f, static_cast<float>(source_size.width),
                0.0f, static_cast<float>(source_size.height),
                -100.0f, 100.0f
            );
			
			// Clear render texture and perform the render
			vec4 clear_color = {};
			vec4_zero(&clear_color);
			gs_clear(GS_CLEAR_COLOR, &clear_color, 0.0f, 0);

			if (target == parent && allow_direct_render)
				obs_source_default_render(target);
			else
				obs_source_video_render(target);

			// Restore render state
			gs_matrix_pop();
			gs_projection_pop();
			gs_viewport_pop();
			gs_blend_state_pop();
			
			// Restore render targets
			gs_set_render_target(
				prev_render_target,
				prev_z_stencil_target
			);

			return true;
		}
		return false;
	}

//---------------------------------------------------------------------------------------------------------------------

	DefaultEffect::DefaultEffect()
		: OBSEffect(obs_get_base_effect(OBS_EFFECT_DEFAULT))
	{}

//---------------------------------------------------------------------------------------------------------------------

}