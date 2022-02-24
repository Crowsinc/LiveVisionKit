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

#include <vector>

#include "LiveVisionKit.hpp"
#include "OBS/Interop/VisionFilter.hpp"

namespace lvk
{

	class LCFilter : public VisionFilter
	{
	public:

		static obs_properties_t* Properties();

		static void LoadDefaults(obs_data_t* settings);

		LCFilter(obs_source_t* context);

		void configure(obs_data_t* settings);

		bool validate() const;

	private:

		void prepare_undistort_maps(cv::UMat& frame);

		virtual void filter(cv::UMat& frame) override;

	private:

		obs_source_t* m_Context;

		bool m_CorrectDistortion;

		std::string m_Profile;
		CameraParameters m_Parameters;

		cv::UMat m_UndistortFrame;
		cv::UMat m_UndistortMap, m_AuxUndistortMap;
		cv::Rect m_UndistortCrop;
	};

}
