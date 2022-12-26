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

#include <obs-module.h>
#include <LiveVisionKit.hpp>

namespace lvk
{

	class FSRFilter
	{
	public:

		static obs_properties_t* Properties();

		static void LoadDefaults(obs_data_t* settings);

		explicit FSRFilter(obs_source_t* context);

		void tick();

		void render();

		void configure(obs_data_t* settings);

		uint32_t width() const;

		uint32_t height() const;

		bool validate() const;

	private:

		obs_source_t* m_Context = nullptr;

		cv::Size m_RequestedSize;
		float m_SizeMultiplier = 1.0f;
		cv::Size m_InputSize, m_OutputSize;

		cv::Rect m_ScalingRegion;
		cv::Size m_TLCrop, m_BRCrop;

		bool m_MatchCanvasSize = false;
		bool m_MatchSourceSize = false;
		bool m_MaintainAspectRatio = true;
	};

}
