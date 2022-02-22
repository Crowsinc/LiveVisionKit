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

	class ADBFilter : public VisionFilter
	{
	public:

		static obs_properties_t* Properties();

		static void LoadDefaults(obs_data_t* settings);

		ADBFilter(obs_source_t* context);

		void configure(obs_data_t* settings);

		bool validate() const;

	private:

		virtual void filter(cv::UMat& frame) override;

		void draw_debug_info(cv::UMat& frame, const uint64_t frame_time_ns);

	private:

		obs_source_t* m_Context;

		bool m_TestMode;
		uint32_t m_DetectionLevels;

		cv::UMat m_BlockGrid, m_ChannelMask, m_BlockMask;
		cv::UMat m_Buffer, m_DeblockBuffer, m_FloatBuffer;
		cv::UMat m_KeepBlendMap, m_DeblockBlendMap;

	};

}
