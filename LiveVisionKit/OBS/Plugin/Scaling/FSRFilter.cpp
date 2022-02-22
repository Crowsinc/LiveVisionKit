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

#include "FSRFilter.hpp"

#include <string>

#define A_CPU 1
#include "../Effects/ffx_a.h"
#include "../Effects/ffx_fsr1.h"

// NOTE: The FSR effect fully supports RCAS, but is no longer ran alongside EASU
// in favour of running the standalone CAS filter instead. Not to mention performing
// a multipass render through OBS is currently a bit of an ugly hack, more so when the
// first pass must also perform scaling of the render target.

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	constexpr auto PROP_OUTPUT_SIZE   = "OUTPUT_SIZE";
	constexpr auto OUTPUT_SIZE_CANVAS = "CANVAS";
	constexpr auto OUTPUT_SIZE_2160P  = "2160P";
	constexpr auto OUTPUT_SIZE_1440P  = "1440P";
	constexpr auto OUTPUT_SIZE_1080P  = "1080P";
	constexpr auto OUTPUT_SIZE_720P   = "720P";
	constexpr auto OUTPUT_SIZE_DEFAULT = OUTPUT_SIZE_CANVAS;

//---------------------------------------------------------------------------------------------------------------------

	obs_properties_t* FSRFilter::Properties()
	{
		obs_properties_t* properties = obs_properties_create();

		auto property = obs_properties_add_list(
				properties,
				PROP_OUTPUT_SIZE,
				"Output Size",
				OBS_COMBO_TYPE_LIST,
				OBS_COMBO_FORMAT_STRING
		);

		obs_property_list_add_string(property, "Canvas Size", OUTPUT_SIZE_CANVAS);
		obs_property_list_add_string(property, "3840x2160   (2160p)", OUTPUT_SIZE_2160P);
		obs_property_list_add_string(property, "2560x1440   (1440p)", OUTPUT_SIZE_1440P);
		obs_property_list_add_string(property, "1920x1080   (1080p)", OUTPUT_SIZE_1080P);
		obs_property_list_add_string(property, "1280x720     (720p)", OUTPUT_SIZE_720P);

		return properties;
	}

//---------------------------------------------------------------------------------------------------------------------

	void FSRFilter::LoadDefaults(obs_data_t* settings)
	{
		LVK_ASSERT(settings != nullptr);

		obs_data_set_default_string(settings, PROP_OUTPUT_SIZE, OUTPUT_SIZE_DEFAULT);
	}

//---------------------------------------------------------------------------------------------------------------------

	FSRFilter* FSRFilter::Create(obs_source_t* context, obs_data_t* settings)
	{
		LVK_ASSERT(context != nullptr && settings != nullptr);

		auto filter = new FSRFilter(context);

		if(!filter->validate())
		{
			delete filter;
			return nullptr;
		}

		filter->configure(settings);

		return filter;
	}

//---------------------------------------------------------------------------------------------------------------------

	FSRFilter::FSRFilter(obs_source_t* context)
		: m_Context(context),
		  m_Shader(nullptr),
		  m_EASUMatchCanvas(false),
		  m_OutputSizeParam(nullptr),
		  m_EASUConstParam0(nullptr),
		  m_EASUConstParam1(nullptr),
		  m_EASUConstParam2(nullptr),
		  m_EASUConstParam3(nullptr)
	{
		LVK_ASSERT(context != nullptr);

		char* shader_path = obs_module_file("effects/fsr.effect");
		if(shader_path != nullptr)
		{
			obs_enter_graphics();

			m_Shader = gs_effect_create_from_file(shader_path, nullptr);
			bfree(shader_path);

			if(m_Shader)
			{
				m_OutputSizeParam = gs_effect_get_param_by_name(m_Shader, "output_size");
				m_EASUConstParam0 = gs_effect_get_param_by_name(m_Shader, "easu_const_0");
				m_EASUConstParam1 = gs_effect_get_param_by_name(m_Shader, "easu_const_1");
				m_EASUConstParam2 = gs_effect_get_param_by_name(m_Shader, "easu_const_2");
				m_EASUConstParam3 = gs_effect_get_param_by_name(m_Shader, "easu_const_3");
			}

			obs_leave_graphics();
		}

		// NOTE: Must get configured() by Create() before first update
		vec2_zero(&m_NewOutputSize);
		vec2_zero(&m_OutputSize);
		vec2_zero(&m_InputSize);
	}

//---------------------------------------------------------------------------------------------------------------------

	FSRFilter::~FSRFilter()
	{
		obs_enter_graphics();

		if(m_Shader != nullptr)
			gs_effect_destroy(m_Shader);

		obs_leave_graphics();
	}

//---------------------------------------------------------------------------------------------------------------------

	void FSRFilter::configure(obs_data_t* settings)
	{
		LVK_ASSERT(settings != nullptr);

		m_EASUMatchCanvas = false;

		const std::string output_size = obs_data_get_string(settings, PROP_OUTPUT_SIZE);
		if(output_size == OUTPUT_SIZE_CANVAS)
			m_EASUMatchCanvas = true;
		else if(output_size == OUTPUT_SIZE_2160P)
			vec2_set(&m_NewOutputSize, 3840, 2160);
		else if(output_size == OUTPUT_SIZE_1440P)
			vec2_set(&m_NewOutputSize, 2560, 1440);
		else if(output_size == OUTPUT_SIZE_1080P)
			vec2_set(&m_NewOutputSize, 1920, 1080);
		else if(output_size == OUTPUT_SIZE_720P)
			vec2_set(&m_NewOutputSize, 1280, 720);
	}

//---------------------------------------------------------------------------------------------------------------------

	bool FSRFilter::update_scaling()
	{
		const auto filter_target = obs_filter_get_target(m_Context);
		const uint32_t input_width = obs_source_get_base_width(filter_target);
		const uint32_t input_height = obs_source_get_base_height(filter_target);

		if(m_EASUMatchCanvas)
		{
			obs_video_info video_info;
			obs_get_video_info(&video_info);

			vec2_set(&m_NewOutputSize, video_info.base_width, video_info.base_height);
		}

		const bool easu_outdated = m_OutputSize.x != m_NewOutputSize.x
								|| m_OutputSize.y != m_NewOutputSize.y
								|| input_width != m_InputSize.x
								|| input_height != m_InputSize.y;

		m_OutputSize = m_NewOutputSize;

		if(easu_outdated)
		{
			vec2_set(&m_InputSize, input_width, input_height);

			// NOTE: The EASU constants are a vector of four uint32_t but their bits actually represent
			// floats. Normally this conversion happens in the FSR shader. However due to compatibility
			// issues, we perform the conversion on the CPU instead. So here we pass in float pointers,
			// casted to uint32_t pointers to facilitate the uint32_t to float re-interpretation.
			FsrEasuCon((AU1*)m_EASUConst0.ptr, (AU1*)m_EASUConst1.ptr, (AU1*)m_EASUConst2.ptr, (AU1*)m_EASUConst3.ptr,
					m_InputSize.x, m_InputSize.y, m_InputSize.x, m_InputSize.y,
					m_OutputSize.x, m_OutputSize.y);
		}

		return input_width != m_OutputSize.x || input_height != m_OutputSize.y;
	}

//---------------------------------------------------------------------------------------------------------------------

	void FSRFilter::render()
	{
		if(update_scaling() && obs_source_process_filter_begin(m_Context, GS_RGBA, OBS_ALLOW_DIRECT_RENDERING))
		{
			gs_effect_set_vec2(m_OutputSizeParam, &m_OutputSize);
			gs_effect_set_vec4(m_EASUConstParam0, &m_EASUConst0);
			gs_effect_set_vec4(m_EASUConstParam1, &m_EASUConst1);
			gs_effect_set_vec4(m_EASUConstParam2, &m_EASUConst2);
			gs_effect_set_vec4(m_EASUConstParam3, &m_EASUConst3);

			obs_source_process_filter_tech_end(m_Context, m_Shader, m_OutputSize.x, m_OutputSize.y, "EASU");
		}
		else obs_source_skip_video_filter(m_Context);
	}

//---------------------------------------------------------------------------------------------------------------------

	uint32_t FSRFilter::width() const
	{
		return m_OutputSize.x;
	}

//---------------------------------------------------------------------------------------------------------------------

	uint32_t FSRFilter::height() const
	{
		return m_OutputSize.y;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool FSRFilter::validate() const
	{
		return m_Context != nullptr
			&& m_Shader != nullptr
			&& m_OutputSizeParam != nullptr
			&& m_EASUConstParam0 != nullptr
			&& m_EASUConstParam1 != nullptr
			&& m_EASUConstParam2 != nullptr
			&& m_EASUConstParam3 != nullptr;
	}

//---------------------------------------------------------------------------------------------------------------------
}
