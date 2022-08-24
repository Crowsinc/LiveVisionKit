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

#include "CASEffect.hpp"

#include "Math/Math.hpp"
#include "OBS/Utility/Logging.hpp"

#define A_CPU 1
#include "../Data/effects/ffx_a.h"
#include "../Data/effects/ffx_cas.h"

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	CASEffect::CASEffect()
		: OBSEffect("cas")
	{
		if (handle() != nullptr)
		{
			obs_enter_graphics();

			m_OutputSizeParam = load_param("output_size");
			m_CASConstParam = load_param("cas_const_1");

			obs_leave_graphics();
		}
	
		log::error_if(!validate(), "CAS effect failed to validate");
	}

//---------------------------------------------------------------------------------------------------------------------

	bool CASEffect::should_skip(
		const cv::Size source_size,
		const cv::Size render_size,
		const float sharpness
	) const
	{
		LVK_ASSERT(between(sharpness, 0.0f, 1.0f));
		return false;
	}

//---------------------------------------------------------------------------------------------------------------------

	const char* CASEffect::configure(
		const cv::Size source_size,
		const cv::Size render_size,
		const float sharpness
	)
	{
		vec2 size_param;
		vec2_set(&size_param, render_size.width, render_size.height);
		gs_effect_set_vec2(m_OutputSizeParam, &size_param);

		// NOTE: The CAS constant is a vector of four uint32_t but its bits actually represent floats.
		// Normally this conversion happens in the CAS shader. However due to compatibility issues,
		// we perform the conversion on the CPU instead. So here we pass in float pointers, casted
		// to uint32_t pointers to facilitate the uint32_t to float re-interpretation. Additionally
		// we only care about const1 and the sharpness input, as the rest are for the CAS scaling
		// functionality which isn't used.

		vec4 tmp = {0}, const_1 = {0};
		CasSetup((AU1*)tmp.ptr, (AU1*)const_1.ptr, sharpness, 0.0f, 0.0f, 0.0f, 0.0f);
		gs_effect_set_vec4(m_CASConstParam, &const_1);

		return "Draw";
	}

//---------------------------------------------------------------------------------------------------------------------

	bool CASEffect::validate() const
	{
		return m_OutputSizeParam != nullptr
			&& m_CASConstParam != nullptr;
	}

//---------------------------------------------------------------------------------------------------------------------


}