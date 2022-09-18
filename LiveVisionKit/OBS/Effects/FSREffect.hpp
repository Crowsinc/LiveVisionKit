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
	
	class FSREffect : public OBSEffect<FSREffect, const cv::Rect& /* scaling region */>
	{
	public:

		FSREffect();

	private:

		const char* configure(
			const cv::Size source_size,
			const cv::Size render_size,
			const cv::Rect& region
		) override;

		bool should_skip(
			const cv::Size source_size,
			const cv::Size render_size,
			const cv::Rect& region
		) const override;

		bool validate() const override;

	private:

		gs_eparam_t* m_InputSizeParam = nullptr;
		gs_eparam_t* m_OutputSizeParam = nullptr;
		gs_eparam_t* m_RegionUVOffsetParam = nullptr;
		std::array<vec4, 4> m_EASUConstants = {vec4{}, vec4{}, vec4{}, vec4{}};
		std::array<gs_eparam_t*, 4> m_EASUParams = {nullptr, nullptr, nullptr, nullptr};
	};

}
