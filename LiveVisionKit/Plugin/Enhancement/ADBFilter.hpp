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

#include "../../LiveVisionKit.hpp"

namespace lvk
{

	class ADBFilter
	{
	public:

		static obs_properties_t* Properties();

		static void LoadDefaults(obs_data_t* settings);

		static ADBFilter* Create(obs_source_t* context, obs_data_t* settings);

		void configure(obs_data_t* settings);

		obs_source_frame* process(obs_source_frame* obs_frame);

		void reset();

	private:

		obs_source_t* m_Context;

		bool m_TestMode;
		int m_KeepThreshold;

		cv::UMat m_Frame, m_FilteredFrame;
		cv::UMat m_Buffer, m_FloatBuffer;
		cv::UMat m_BlockGrid, m_GridMask;
		cv::UMat m_KeepBlendMap, m_DeblockBlendMap;


		ADBFilter(obs_source_t* context);

		cv::UMat draw_debug_info(cv::UMat& frame, const uint64_t frame_time_ns);

		bool validate() const;

	};

}
