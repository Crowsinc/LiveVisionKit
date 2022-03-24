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
	constexpr auto OUTPUT_SIZE_SOURCE = "Original Size";
	constexpr auto OUTPUT_SIZE_CANVAS = "Canvas Size";
	constexpr auto OUTPUT_SIZE_DEFAULT = OUTPUT_SIZE_SOURCE;

	const std::vector<std::string> OUTPUT_SIZES = {
		"3840x2160", "2560x1440", "1920x1080", "1280x720", "x2", "x0.5"
	};

	constexpr auto OUTPUT_MAX_DIMENSION = 4096;
	constexpr auto OUTPUT_MIN_DIMENSION = 1;
	constexpr auto OUTPUT_DEFAULT_MULTIPLIER = 1.0f;


	constexpr auto PROP_CROP_GROUP = "CROP_GROUP";

	constexpr auto PROP_MAINTAIN_ASPECT = "MAINTAIN_ASPECT_RATIO";
	constexpr auto MAINTAIN_ASPECT_DEFAULT = true;

	constexpr auto PROP_CROP_TOP = "CROP_TOP";
	constexpr auto PROP_CROP_LEFT = "CROP_LEFT";
	constexpr auto PROP_CROP_RIGHT = "CROP_RIGHT";
	constexpr auto PROP_CROP_BOTTOM = "CROP_BOTTOM";

	constexpr auto CROP_MIN = 0;
	constexpr auto CROP_MAX = OUTPUT_MAX_DIMENSION;
	constexpr auto CROP_STEP = 1;
	constexpr auto CROP_DEFAULT = CROP_MIN;


//---------------------------------------------------------------------------------------------------------------------

	obs_properties_t* FSRFilter::Properties()
	{
		obs_properties_t* properties = obs_properties_create();

		auto property = obs_properties_add_list(
			properties,
			PROP_OUTPUT_SIZE,
			"Output Size",
			obs_combo_type::OBS_COMBO_TYPE_EDITABLE,
			obs_combo_format::OBS_COMBO_FORMAT_STRING
		);

		obs_property_list_add_string(property, OUTPUT_SIZE_SOURCE, OUTPUT_SIZE_SOURCE);
		obs_property_list_add_string(property, OUTPUT_SIZE_CANVAS, OUTPUT_SIZE_CANVAS);

		for(const auto& size : OUTPUT_SIZES)
			obs_property_list_add_string(property, size.c_str(), size.c_str());

		obs_properties_add_bool(
			properties,
			PROP_MAINTAIN_ASPECT,
			"Maintain Aspect Ratio"
		);

		obs_properties_t* crop_properties = obs_properties_create();
		obs_properties_add_group(
			properties,
			PROP_CROP_GROUP,
			"Crop",
			obs_group_type::OBS_GROUP_NORMAL,
			crop_properties
		);

		obs_properties_add_int(
			crop_properties,
			PROP_CROP_TOP,
			"Top",
			CROP_MIN,
			CROP_MAX,
			CROP_STEP
		);

		obs_properties_add_int(
			crop_properties,
			PROP_CROP_BOTTOM,
			"Bottom",
			CROP_MIN,
			CROP_MAX,
			CROP_STEP
		);

		obs_properties_add_int(
			crop_properties,
			PROP_CROP_LEFT,
			"Left",
			CROP_MIN,
			CROP_MAX,
			CROP_STEP
		);

		obs_properties_add_int(
			crop_properties,
			PROP_CROP_RIGHT,
			"Right",
			CROP_MIN,
			CROP_MAX,
			CROP_STEP
		);

		return properties;
	}

//---------------------------------------------------------------------------------------------------------------------

	void FSRFilter::LoadDefaults(obs_data_t* settings)
	{
		LVK_ASSERT(settings != nullptr);

		obs_data_set_default_string(settings, PROP_OUTPUT_SIZE, OUTPUT_SIZE_DEFAULT);
		obs_data_set_default_bool(settings, PROP_MAINTAIN_ASPECT, MAINTAIN_ASPECT_DEFAULT);

		obs_data_set_default_int(settings, PROP_CROP_TOP, CROP_DEFAULT);
		obs_data_set_default_int(settings, PROP_CROP_LEFT, CROP_DEFAULT);
		obs_data_set_default_int(settings, PROP_CROP_RIGHT, CROP_DEFAULT);
		obs_data_set_default_int(settings, PROP_CROP_BOTTOM, CROP_DEFAULT);
	}

//---------------------------------------------------------------------------------------------------------------------

	void FSRFilter::configure(obs_data_t* settings)
	{
		LVK_ASSERT(settings != nullptr);

		m_MatchCanvasSize = m_MatchSourceSize = false;
		m_SizeMultiplier = OUTPUT_DEFAULT_MULTIPLIER;

		const std::string output_command = obs_data_get_string(settings, PROP_OUTPUT_SIZE);
		if(output_command == OUTPUT_SIZE_CANVAS)
			m_MatchCanvasSize = true;
		else if(output_command == OUTPUT_SIZE_SOURCE)
			m_MatchSourceSize = true;
		else
		{
			std::vector<float> tokens = split<float>(output_command, 'x', [](auto _, float& value, const bool fail){
				return !fail && value > 0;
			});

			// Interpret custom resolution
			if(tokens.size() == 2 || tokens.size() == 3)
			{
				// Interpret optional multiplier
				if(tokens.size() == 3)
					m_SizeMultiplier = tokens[2];

				m_OutputSize = {
					static_cast<int>(tokens[0] * m_SizeMultiplier),
					static_cast<int>(tokens[1] * m_SizeMultiplier)
				};
			}

			// Interpret solo multiplier
			if(tokens.size() == 1)
			{
				m_SizeMultiplier = tokens[0];
				m_MatchSourceSize = true;
			}
		}

		m_MaintainAspectRatio = obs_data_get_bool(settings, PROP_MAINTAIN_ASPECT);
		m_TLCrop.width = obs_data_get_int(settings, PROP_CROP_LEFT);
		m_TLCrop.height = obs_data_get_int(settings, PROP_CROP_TOP);
		m_BRCrop.width = obs_data_get_int(settings, PROP_CROP_RIGHT);
		m_BRCrop.height = obs_data_get_int(settings, PROP_CROP_BOTTOM);
	}

//---------------------------------------------------------------------------------------------------------------------

	FSRFilter::FSRFilter(obs_source_t* context)
		: m_Context(context),
		  m_OutputSize(0, 0),
		  m_MatchCanvasSize(false),
		  m_MatchSourceSize(false),
		  m_SizeMultiplier(1.0),
		  m_TLCrop(0,0),
		  m_BRCrop(0,0),
		  m_MaintainAspectRatio(true)
	{
		LVK_ASSERT(context != nullptr);
	}

//---------------------------------------------------------------------------------------------------------------------

	void FSRFilter::render()
	{
		const auto filter_target = obs_filter_get_target(m_Context);
		const cv::Size input_size(
			obs_source_get_base_width(filter_target),
			obs_source_get_base_height(filter_target)
		);


		// Update output size
		if(m_MatchSourceSize)
			m_OutputSize = cv::Size2f(input_size) * m_SizeMultiplier;
		else if(m_MatchCanvasSize)
		{
			obs_video_info video_info;
			obs_get_video_info(&video_info);

			m_OutputSize.width = video_info.base_width;
			m_OutputSize.height = video_info.base_height;
		}

		const cv::Rect crop_region(m_TLCrop, input_size - m_BRCrop - m_TLCrop);
		const bool crop_valid = crop_region.x < crop_region.br().x
							 && crop_region.y < crop_region.br().y
							 && crop_region.width > 0
							 && crop_region.height > 0;

		if(m_MaintainAspectRatio && crop_valid)
		{
			const auto scale = std::min<float>(
				static_cast<float>(m_OutputSize.width) / crop_region.width,
				static_cast<float>(m_OutputSize.height) / crop_region.height
			);
			m_OutputSize = cv::Size2f(crop_region.size()) * scale;
		}

		//TODO: Fix render bug when only one crop dimension is applied
		//TODO: Silently clamp to valid dimensions to avoid errors with uninitialised sizes.
		// Should probably make it so we can't have invalid sizes to begin with.
		m_OutputSize.width = std::clamp(m_OutputSize.width, OUTPUT_MIN_DIMENSION, OUTPUT_MAX_DIMENSION);
		m_OutputSize.height = std::clamp(m_OutputSize.height, OUTPUT_MIN_DIMENSION, OUTPUT_MAX_DIMENSION);

		// Only crop if valid
		if(crop_valid)
			FSREffect::Get().scale(m_Context, crop_region, m_OutputSize);
		else
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
