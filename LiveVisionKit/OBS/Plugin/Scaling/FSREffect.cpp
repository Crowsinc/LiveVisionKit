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

#include "FSREffect.hpp"

#define A_CPU 1
#include "../Effects/ffx_a.h"
#include "../Effects/ffx_fsr1.h"

// NOTE: The FSR effect shader fully supports RCAS, but is no longer ran alongside EASU
// in favour of running the standalone CAS effect instead. Performing a multipass render
// through OBS is currently a bit of an ugly hack, more so when the first pass must also
// perform scaling of the render target.

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	FSREffect::FSREffect()
		: OBSEffect("fsr")
	{
		if(handle() != nullptr)
		{
			obs_enter_graphics();
			m_InputSizeParam = gs_effect_get_param_by_name(handle(), "input_size");
			m_OutputSizeParam = gs_effect_get_param_by_name(handle(), "output_size");
			m_RegionUVOffsetParam = gs_effect_get_param_by_name(handle(), "region_uv_offset");

			m_EASUParams[0] = gs_effect_get_param_by_name(handle(), "easu_const_0");
			m_EASUParams[1] = gs_effect_get_param_by_name(handle(), "easu_const_1");
			m_EASUParams[2] = gs_effect_get_param_by_name(handle(), "easu_const_2");
			m_EASUParams[3] = gs_effect_get_param_by_name(handle(), "easu_const_3");
			obs_leave_graphics();
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void FSREffect::scale(obs_source_t* context, const cv::Rect& region, const cv::Size& output_size)
	{
		const auto filter_target = obs_filter_get_target(context);
		const cv::Size input_size(
			obs_source_get_base_width(filter_target),
			obs_source_get_base_height(filter_target)
		);

		LVK_ASSERT(output_size.width >= 0 && output_size.height >= 0);
		LVK_ASSERT(between(region.x, 0, input_size.width) && between(region.br().x, region.x, input_size.width));
		LVK_ASSERT(between(region.y, 0, input_size.height) && between(region.br().y, region.y, input_size.height));

		// Silently skip if no scaling/cropping is needed or any of the sizes are zero.
		// Dont crash on zero sizing because there are scenarios where it is expected or reasonable input.
		const bool invalid_output = input_size.area() == 0 || region.area() == 0 || output_size.area() == 0;
		if((input_size == output_size && region.size() == input_size) || invalid_output)
		{
			obs_source_skip_video_filter(context);
			return;
		}

		if(obs_source_process_filter_begin(context, GS_RGBA, OBS_ALLOW_DIRECT_RENDERING))
		{
			// Input Size
			vec2 param;
			vec2_set(&param, input_size.width, input_size.height);
			gs_effect_set_vec2(m_InputSizeParam, &param);

			// Output Size
			vec2_set(&param, output_size.width, output_size.height);
			gs_effect_set_vec2(m_OutputSizeParam, &param);

			// Region UV Offset
			vec2_set(
				&param,
				static_cast<float>(region.x) / input_size.width,
				static_cast<float>(region.y) / input_size.height
			);
			gs_effect_set_vec2(m_RegionUVOffsetParam, &param);

			// EASU constants

			// NOTE: The constants are a vector of four uint32_t but their bits actually represent
			// floats. Normally this conversion happens in the FSR shader. However due to compatibility
			// issues, we perform the conversion on the CPU instead. So we pass in float pointers,
			// casted to uint32_t pointers to facilitate the uint32_t to float re-interpretation.
			FsrEasuCon(
				(AU1*)m_EASUConstants[0].ptr,
				(AU1*)m_EASUConstants[1].ptr,
				(AU1*)m_EASUConstants[2].ptr,
				(AU1*)m_EASUConstants[3].ptr,
				region.width,
				region.height,
				input_size.width,
				input_size.height,
				output_size.width,
				output_size.height
			);

			for(size_t i = 0; i < m_EASUParams.size(); i++)
				gs_effect_set_vec4(m_EASUParams[i], &m_EASUConstants[i]);

			obs_source_process_filter_tech_end(context, handle(), output_size.width, output_size.height, "EASU");
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	bool FSREffect::validate() const
	{
		return m_OutputSizeParam != nullptr
			&& m_InputSizeParam != nullptr
			&& m_RegionUVOffsetParam != nullptr
			&& m_EASUParams[0] != nullptr
			&& m_EASUParams[1] != nullptr
			&& m_EASUParams[2] != nullptr
			&& m_EASUParams[3] != nullptr;
	}

//---------------------------------------------------------------------------------------------------------------------

}
