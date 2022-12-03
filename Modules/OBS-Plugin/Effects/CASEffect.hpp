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

#pragma once

#include "OBSEffect.hpp"

namespace lvk
{

	class CASEffect final : public OBSEffect<CASEffect, const float /* sharpness */>
	{
	public:

		CASEffect();

	private:

		const char* configure(
			const cv::Size source_size,
			const cv::Size render_size,
			const float sharpness
		) override;

		bool should_skip(
			const cv::Size source_size,
			const cv::Size render_size,
			const float sharpness
		) const override;

		[[nodiscard]] bool validate() const override;

	private:
		gs_eparam_t* m_CASConstParam = nullptr;
		gs_eparam_t* m_OutputSizeParam = nullptr;
	};


}
