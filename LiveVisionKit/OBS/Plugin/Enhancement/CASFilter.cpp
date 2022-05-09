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

#include "OBS/Plugin/Effects/CASEffect.hpp"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	constexpr auto PROP_SHARPNESS = "OUTPUT_SHARPNESS";
	constexpr auto SHARPNESS_MIN = 0.0f;
	constexpr auto SHARPNESS_MAX = 1.0f;
	constexpr auto SHARPNESS_STEP = 0.05f;
	constexpr auto SHARPNESS_DEFAULT = 0.8f;

//---------------------------------------------------------------------------------------------------------------------

	obs_properties_t* CASFilter::Properties()
	{
		obs_properties_t* properties = obs_properties_create();

		obs_properties_add_float_slider(
			properties,
			PROP_SHARPNESS,
			"Sharpness",
			SHARPNESS_MIN,
			SHARPNESS_MAX,
			SHARPNESS_STEP
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

		m_Sharpness = obs_data_get_double(settings, PROP_SHARPNESS);
	}

//---------------------------------------------------------------------------------------------------------------------

	CASFilter::CASFilter(obs_source_t* context)
		: m_Context(context)
	{
		LVK_ASSERT(context != nullptr);
	}

//---------------------------------------------------------------------------------------------------------------------

	void CASFilter::render()
	{
		const auto filter_target = obs_filter_get_target(m_Context);
		
		m_OutputSize.width = obs_source_get_base_width(filter_target);
		m_OutputSize.height = obs_source_get_base_height(filter_target);

		if(!CASEffect::Render(m_Context, m_OutputSize, m_Sharpness))
			obs_source_skip_video_filter(m_Context);
	}

//---------------------------------------------------------------------------------------------------------------------

	uint32_t CASFilter::width() const
	{
		return m_OutputSize.width;
	}

//---------------------------------------------------------------------------------------------------------------------

	uint32_t CASFilter::height() const
	{
		return m_OutputSize.height;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool CASFilter::validate() const
	{
		// Ensure we have no nulls for key filter members
		return m_Context != nullptr
			&& CASEffect::Validate();
	}

//---------------------------------------------------------------------------------------------------------------------
}
