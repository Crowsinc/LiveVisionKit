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

#include "FSREffect.hpp"

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

	void FSRFilter::configure(obs_data_t* settings)
	{
		LVK_ASSERT(settings != nullptr);

		m_MatchCanvasSize = false;

		const std::string output_size = obs_data_get_string(settings, PROP_OUTPUT_SIZE);
		if(output_size == OUTPUT_SIZE_CANVAS)
			m_MatchCanvasSize = true;
		else if(output_size == OUTPUT_SIZE_2160P)
			m_OutputSize = {3840, 2160};
		else if(output_size == OUTPUT_SIZE_1440P)
			m_OutputSize = {2560, 1440};
		else if(output_size == OUTPUT_SIZE_1080P)
			m_OutputSize = {1920, 1080};
		else if(output_size == OUTPUT_SIZE_720P)
			m_OutputSize = {1280, 720};
	}

//---------------------------------------------------------------------------------------------------------------------

	FSRFilter::FSRFilter(obs_source_t* context)
		: m_Context(context),
		  m_MatchCanvasSize(false),
		  m_OutputSize(0, 0)
	{
		LVK_ASSERT(context != nullptr);
	}

//---------------------------------------------------------------------------------------------------------------------

	void FSRFilter::render()
	{
		if(m_MatchCanvasSize)
		{
			obs_video_info video_info;
			obs_get_video_info(&video_info);

			m_OutputSize.width = video_info.base_width;
			m_OutputSize.height = video_info.base_height;
		}

		FSREffect::Get().scale(m_Context, m_OutputSize);
	}

//---------------------------------------------------------------------------------------------------------------------

	uint32_t FSRFilter::width() const
	{
		return m_OutputSize.width;
	}

//---------------------------------------------------------------------------------------------------------------------

	uint32_t FSRFilter::height() const
	{
		return m_OutputSize.height;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool FSRFilter::validate() const
	{
		return m_Context != nullptr
			&& FSREffect::Validate();
	}

//---------------------------------------------------------------------------------------------------------------------
}
