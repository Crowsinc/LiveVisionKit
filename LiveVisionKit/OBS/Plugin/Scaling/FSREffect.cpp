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
			m_OutputSizeParam = gs_effect_get_param_by_name(handle(), "output_size");
			m_InputSizeParam = gs_effect_get_param_by_name(handle(), "input_size");
			m_UVOffsetParam = gs_effect_get_param_by_name(handle(), "uv_offset");
			m_UVScaleParam = gs_effect_get_param_by_name(handle(), "uv_scale");

			m_EASUParams[0] = gs_effect_get_param_by_name(handle(), "easu_const_0");
			m_EASUParams[1] = gs_effect_get_param_by_name(handle(), "easu_const_1");
			m_EASUParams[2] = gs_effect_get_param_by_name(handle(), "easu_const_2");
			m_EASUParams[3] = gs_effect_get_param_by_name(handle(), "easu_const_3");
			obs_leave_graphics();
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void FSREffect::scale(obs_source_t* context, const cv::Size& output_size)
	{
		const auto filter_target = obs_filter_get_target(context);

		cv::Rect region(
			0, 0,
			obs_source_get_base_width(filter_target),
			obs_source_get_base_height(filter_target)
		);


		scale(context, region, output_size);
	}

//---------------------------------------------------------------------------------------------------------------------

	void FSREffect::scale(obs_source_t* context, const cv::Rect& region, const cv::Size& output_size)
	{
		LVK_ASSERT(region.x >= 0 && region.y >= 0 && region.width >= 0 && region.height >= 0);

		if(region.size() == output_size || region.area() == 0)
			obs_source_skip_video_filter(context);

		if(obs_source_process_filter_begin(context, GS_RGBA, OBS_NO_DIRECT_RENDERING))
		{
			prepare_easu(region, output_size);

			obs_source_process_filter_tech_end(context, handle(), output_size.width, output_size.height, "EASU");
		}
		else obs_source_skip_video_filter(context);
	}

//---------------------------------------------------------------------------------------------------------------------

	void FSREffect::prepare_easu(const cv::Rect& region, const cv::Size& output_size)
	{
		vec2 param_val;

		// Output Size
		vec2_set(&param_val, output_size.width, output_size.height);
		gs_effect_set_vec2(m_OutputSizeParam, &param_val);

		// Input Size
		vec2_set(&param_val, region.width, region.height);
		gs_effect_set_vec2(m_InputSizeParam, &param_val);

		// Input UV Offset
		vec2_set(
			&param_val,
			static_cast<float>(region.x) / output_size.width,
			static_cast<float>(region.y) / output_size.height
		);
		gs_effect_set_vec2(m_UVOffsetParam, &param_val);

		// Input UV Scale
		vec2_set(
			&param_val,
			std::min(static_cast<float>(region.width) / output_size.width, 1.0f),
			std::min(static_cast<float>(region.height) / output_size.height, 1.0f)
		);
		gs_effect_set_vec2(m_UVScaleParam, &param_val);

		// NOTE: The EASU constants are a vector of four uint32_t but their bits actually represent
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
			region.width,
			region.height,
			output_size.width,
			output_size.height
		);

		for(size_t i = 0; i < m_EASUParams.size(); i++)
			gs_effect_set_vec4(m_EASUParams[i], &m_EASUConstants[i]);
	}

//---------------------------------------------------------------------------------------------------------------------

	bool FSREffect::validate() const
	{
		return m_OutputSizeParam != nullptr
			&& m_UVOffsetParam != nullptr
			&& m_UVScaleParam != nullptr
			&& m_EASUParams[0] != nullptr
			&& m_EASUParams[1] != nullptr
			&& m_EASUParams[2] != nullptr
			&& m_EASUParams[3] != nullptr;
	}

//---------------------------------------------------------------------------------------------------------------------

}
