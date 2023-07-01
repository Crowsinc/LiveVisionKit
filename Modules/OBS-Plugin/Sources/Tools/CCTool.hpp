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

#include <util/config-file.h>
#include <LiveVisionKit.hpp>
#include <optional>
#include <string>

#include "Interop/VisionFilter.hpp"

namespace lvk
{

	class CCTool : public VisionFilter
	{
	/* Calibration Profile Management */
	public:

		static const std::vector<std::string>& ListProfiles();

		static bool ContainsProfile(const std::string& name);

		static std::optional<CameraParameters> LoadProfile(const std::string& name);

		static bool SaveProfile(const CameraParameters& parameters, const std::string& name);

	/* Camera Calibration Tool  */
	public:

		static obs_properties_t* Properties();

		static void LoadDefaults(obs_data_t* settings);

		explicit CCTool(obs_source_t* context);

		void configure(obs_data_t* settings);

		bool validate() const;

	private:

		static bool on_utility_button(
			obs_properties_t* properties,
			obs_property_t* button,
			void* data
		);

		static bool on_reset_button(
			obs_properties_t* properties,
			obs_property_t* button,
			void* data
		);

		static config_t* load_profile_config();

		void reset();

		void request_capture();

		bool request_calibration();

		bool parameters_valid() const;

		bool calibration_complete() const;

		uint32_t remaining_captures() const;

		std::tuple<std::string, cv::Scalar> generate_calibration_status(const VideoFrame::Format format) const;

		void draw_calibration_hud(OBSFrame& frame) const;

		void filter(OBSFrame& frame) override;

	private:

		obs_source_t* m_Context = nullptr;

		bool m_CaptureNext = false;
		bool m_CalibrateNext = false;
		bool m_CalibrationFail = false;
		bool m_CalibrationSuccess = false;

		cv::UMat m_HoldFrame{cv::UMatUsageFlags::USAGE_ALLOCATE_DEVICE_MEMORY};
		int m_FrameHoldCountdown = 0;

		CameraCalibrator m_Calibrator;
        std::string m_ProfileName;
        uint32_t m_SquareSize;
        cv::Size m_ImageSize;
    };

}
