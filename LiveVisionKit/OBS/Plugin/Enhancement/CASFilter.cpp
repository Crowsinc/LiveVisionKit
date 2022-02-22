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

#include "CASFilter.hpp"

#include <string>

#define A_CPU 1
#include "../Effects/ffx_a.h"
#include "../Effects/ffx_cas.h"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	constexpr auto PROP_SHARPNESS = "OUTPUT_SHARPNESS";
	constexpr auto SHARPNESS_DEFAULT = 0.8f;

//---------------------------------------------------------------------------------------------------------------------

	obs_properties_t* CASFilter::Properties()
	{
		obs_properties_t* properties = obs_properties_create();

		obs_properties_add_float_slider(
				properties,
				PROP_SHARPNESS,
				"Sharpness",
				0,
				1,
				0.05
		);

		return properties;
	}

//---------------------------------------------------------------------------------------------------------------------

	void CASFilter::LoadDefaults(obs_data_t* settings)
	{
		LVK_ASSERT(settings != nullptr);

		obs_data_set_default_double(settings, PROP_SHARPNESS, SHARPNESS_DEFAULT);
	}

//---------------------------------------------------------------------------------------------------------------------

	void CASFilter::configure(obs_data_t* settings)
	{
		LVK_ASSERT(settings != nullptr);

		const float sharpness = obs_data_get_double(settings, PROP_SHARPNESS);

		// NOTE: The CAS constant is a vector of four uint32_t but its bits actually represent floats.
		// Normally this conversion happens in the CAS shader. However due to compatibility issues,
		// we perform the conversion on the CPU instead. So here we pass in float pointers, casted
		// to uint32_t pointers to facilitate the uint32_t to float re-interpretation. Additionally
		// we only care about const1 and the sharpness input, as the rest are for the CAS scaling
		// functionality which isn't used.
		varAU4(const_0);
		CasSetup(const_0, (AU1*)m_CASConst1.ptr, sharpness, 0.0f, 0.0f, 0.0f, 0.0f);
	}

//---------------------------------------------------------------------------------------------------------------------

	CASFilter::CASFilter(obs_source_t* context)
		: m_Context(context),
		  m_Shader(nullptr),
		  m_CASConstParam1(nullptr),
		  m_OutputSizeParam(nullptr)
	{
		LVK_ASSERT(context != nullptr);

		char* shader_path = obs_module_file("effects/cas.effect");
		if(shader_path != nullptr)
		{
			obs_enter_graphics();

			m_Shader = gs_effect_create_from_file(shader_path, nullptr);
			bfree(shader_path);

			if(m_Shader)
			{
				m_OutputSizeParam = gs_effect_get_param_by_name(m_Shader, "output_size");
				m_CASConstParam1 = gs_effect_get_param_by_name(m_Shader, "cas_const_1");
			}

			obs_leave_graphics();
		}

		// NOTE: Must get configured() by Create() before first update
		vec2_zero(&m_OutputSize);
	}

//---------------------------------------------------------------------------------------------------------------------

	CASFilter::~CASFilter()
	{
		obs_enter_graphics();

		if(m_Shader != nullptr)
			gs_effect_destroy(m_Shader);

		obs_leave_graphics();
	}

//---------------------------------------------------------------------------------------------------------------------

	void CASFilter::render()
	{
		const auto filter_target = obs_filter_get_target(m_Context);
		vec2_set(
			&m_OutputSize,
			obs_source_get_base_width(filter_target),
			obs_source_get_base_height(filter_target)
		);

		if(obs_source_process_filter_begin(m_Context, GS_RGBA, OBS_ALLOW_DIRECT_RENDERING))
		{
			gs_effect_set_vec2(m_OutputSizeParam, &m_OutputSize);
			gs_effect_set_vec4(m_CASConstParam1, &m_CASConst1);

			obs_source_process_filter_end(m_Context, m_Shader, m_OutputSize.x, m_OutputSize.y);
		}
		else obs_source_skip_video_filter(m_Context);
	}

//---------------------------------------------------------------------------------------------------------------------

	uint32_t CASFilter::width() const
	{
		return m_OutputSize.x;
	}

//---------------------------------------------------------------------------------------------------------------------

	uint32_t CASFilter::height() const
	{
		return m_OutputSize.y;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool CASFilter::validate() const
	{
		// Ensure we have no nulls for key filter members
		return m_Context != nullptr
			&& m_Shader  != nullptr
			&& m_OutputSizeParam != nullptr
			&& m_CASConstParam1 != nullptr;
	}

//---------------------------------------------------------------------------------------------------------------------
}
