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
#include <filesystem>

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

		static bool advance_calibration(
			obs_properties_t* properties,
			obs_property_t* button,
			void* data
		);

		static bool is_profile_name_valid(const std::string& profile_name);

		void capture_next();

		void reset_captures();

		uint32_t captures_left();

		std::filesystem::path calibrate(const std::string& profile_name);

		virtual void filter(cv::UMat& frame) override;

	private:

		obs_source_t* m_Context;

		bool m_CalibrationMode, m_CaptureNext;
		SlidingBuffer<cv::Mat> m_Captures;

		cv::Mat m_CameraMatrix, m_OptimalCameraMatrix;
		std::vector<double> m_DistortionCoeffs;
		cv::UMat m_CorrectionMap;
	};

}
