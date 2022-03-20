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

#include "LiveVisionKit.hpp"

namespace lvk
{

	class FSREffect : public OBSEffect<FSREffect>
	{
	public:

		FSREffect();

		void scale(obs_source_t* context, const cv::Size& output_size);

		void scale(obs_source_t* context, const cv::Rect& region, const cv::Size& output_size);

		bool validate() const override;

	private:

		void prepare_easu(const cv::Rect& region, const cv::Size& output_size);

	private:

		gs_eparam_t* m_UVScaleParam;
		gs_eparam_t* m_UVOffsetParam;
		gs_eparam_t* m_InputSizeParam;
		gs_eparam_t* m_OutputSizeParam;
		std::array<vec4, 4> m_EASUConstants;
		std::array<gs_eparam_t*, 4> m_EASUParams;
	};

}
