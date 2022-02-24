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

#include "CCTool.hpp"

#include <util/config-file.h>

namespace lvk
{
//---------------------------------------------------------------------------------------------------------------------

	// 2 distances, 4 paths per distance, 5 frames per path
	constexpr auto REQ_CALIBRATION_FRAMES = 2 * 4 * 5;
	constexpr auto CALIBRATION_PATTERN_ROWS = 6;
	constexpr auto CALIBRATION_PATTERN_COLS = 9;

	constexpr auto PROP_UTILITY_BTN = "PROP_UTILITY_BTN";
	constexpr auto UTILITY_BTN_CALIBRATE_TEXT = "Calibrate";
	constexpr auto UTILITY_BTN_CAPTURE_TEXT = "Capture Frame";

	constexpr auto PROP_SQUARE_SIZE = "PROP_SQUARE_SIZE";
	constexpr auto SQUARE_SIZE_MIN = 0;
	constexpr auto SQUARE_SIZE_MAX = 1000;
	constexpr auto SQUARE_SIZE_STEP = 1;
	constexpr auto SQUARE_SIZE_DEFAULT = 0;

	constexpr auto PROP_PROFILE_NAME = "PROP_PROFILE_NAME";
	constexpr auto PROFILE_NAME_DEFAULT = "";

	constexpr auto CONFIG_FILE = "LVK_CALIB_PROFILES.lvkp";

//---------------------------------------------------------------------------------------------------------------------

	std::filesystem::path CCTool::config_file_path()
	{
		char* config_path = obs_module_config_path(CONFIG_FILE);
		std::filesystem::path path = std::string(config_path);
		bfree(config_path);
		return path;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool CCTool::HasProfile(const std::string& name)
	{
		LVK_ASSERT(!name.empty());

		auto config_path = config_file_path();

		config_t* config = nullptr;
		if(config_open(&config, config_path.c_str(), config_open_type::CONFIG_OPEN_EXISTING) == CONFIG_ERROR)
			return false;

		LVK_ASSERT(config != nullptr);

		size_t sections = config_num_sections(config);
		for(size_t i = 0; i < sections; i++)
			if(std::string(config_get_section(config, i)) == name)
			{
				config_close(config);
				return true;
			}

		config_close(config);
		return false;
	}

//---------------------------------------------------------------------------------------------------------------------

	std::optional<CameraParameters> CCTool::LoadProfile(const std::string& name)
	{
		LVK_ASSERT(!name.empty());

		config_t* config = nullptr;
		auto config_path = config_file_path();

		if(!HasProfile(name))
			return {};

		if(config_open(&config, config_path.c_str(), config_open_type::CONFIG_OPEN_EXISTING) == CONFIG_ERROR)
			return {};

		LVK_ASSERT(config != nullptr);

		CameraParameters parameters;

		// Decompose & save camera matrix
		parameters.camera_matrix.at<double>(0, 0) = config_get_double(config, name.c_str(), "fx");
		parameters.camera_matrix.at<double>(1, 1) = config_get_double(config, name.c_str(), "fy");
		parameters.camera_matrix.at<double>(0, 2) = config_get_double(config, name.c_str(), "cx");
		parameters.camera_matrix.at<double>(1, 2) = config_get_double(config, name.c_str(), "cy");

		parameters.distortion_coefficients.push_back(config_get_double(config, name.c_str(), "k1"));
		parameters.distortion_coefficients.push_back(config_get_double(config, name.c_str(), "k2"));
		parameters.distortion_coefficients.push_back(config_get_double(config, name.c_str(), "p1"));
		parameters.distortion_coefficients.push_back(config_get_double(config, name.c_str(), "p2"));
		parameters.distortion_coefficients.push_back(config_get_double(config, name.c_str(), "k3"));

		config_close(config);

		return parameters;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool CCTool::save_profile(const CameraParameters& parameters, const std::string& name)
	{
		LVK_ASSERT(!parameters.camera_matrix.empty() && parameters.camera_matrix.size() == cv::Size(3,3));
		LVK_ASSERT(parameters.distortion_coefficients.size() == 4);

		auto config_path = config_file_path();

		config_t* config = nullptr;
		if(config_open(&config, config_path.c_str(), config_open_type::CONFIG_OPEN_ALWAYS) == CONFIG_ERROR)
			return false;

		LVK_ASSERT(config != nullptr);

		// Decompose & save camera matrix
		const double fx = parameters.camera_matrix.at<double>(0, 0);
		const double fy = parameters.camera_matrix.at<double>(1, 1);
		const double cx = parameters.camera_matrix.at<double>(0, 2);
		const double cy = parameters.camera_matrix.at<double>(1, 2);

		config_set_double(config, name.c_str(), "fx", fx);
		config_set_double(config, name.c_str(), "fy", fy);
		config_set_double(config, name.c_str(), "cx", cx);
		config_set_double(config, name.c_str(), "cy", cy);

		// Decompose & save distortion coefficients
		const double k1 = parameters.distortion_coefficients[0];
		const double k2 = parameters.distortion_coefficients[1];
		const double k3 = parameters.distortion_coefficients[4];
		const double p1 = parameters.distortion_coefficients[2];
		const double p2 = parameters.distortion_coefficients[3];

		config_set_double(config, name.c_str(), "k1", k1);
		config_set_double(config, name.c_str(), "k2", k2);
		config_set_double(config, name.c_str(), "k3", k3);
		config_set_double(config, name.c_str(), "p1", p1);
		config_set_double(config, name.c_str(), "p2", p2);

		config_save(config);
		config_close(config);

		return true;
	}

//---------------------------------------------------------------------------------------------------------------------

	obs_properties_t* CCTool::Properties()
	{
		obs_properties_t* properties = obs_properties_create();

		obs_properties_add_text(
			properties,
			PROP_PROFILE_NAME,
			"Profile Name",
			obs_text_type::OBS_TEXT_DEFAULT
		);

		auto property = obs_properties_add_int(
			properties,
			PROP_SQUARE_SIZE,
			"Square Size (mm)",
			SQUARE_SIZE_MIN,
			SQUARE_SIZE_MAX,
			SQUARE_SIZE_STEP
		);
		obs_property_int_set_suffix(property, "mm");

		obs_properties_add_button(
			properties,
			PROP_UTILITY_BTN,
			UTILITY_BTN_CAPTURE_TEXT,
			CCTool::on_utility_button
		);

		return properties;
	}

//---------------------------------------------------------------------------------------------------------------------

	void CCTool::LoadDefaults(obs_data_t* settings)
	{
		obs_data_set_int(settings, PROP_SQUARE_SIZE, SQUARE_SIZE_DEFAULT);
		obs_data_set_string(settings, PROP_PROFILE_NAME, PROFILE_NAME_DEFAULT);
	}

//---------------------------------------------------------------------------------------------------------------------

	void CCTool::configure(obs_data_t* settings)
	{
		m_ProfileName = obs_data_get_string(settings, PROP_PROFILE_NAME);
		m_SquareSize = obs_data_get_int(settings, PROP_SQUARE_SIZE);

		// Reset calibration complete flag on any configuration changes
		m_CalibrationSuccess = false;
		m_CalibrationFail = false;
	}

//---------------------------------------------------------------------------------------------------------------------

	CCTool::CCTool(obs_source_t* context)
		: VisionFilter(context),
		  m_Context(context),
		  m_Calibrator({CALIBRATION_PATTERN_COLS, CALIBRATION_PATTERN_ROWS})
	{
		reset();
	}

//---------------------------------------------------------------------------------------------------------------------

	void CCTool::reset()
	{
		m_Calibrator.reset();
		m_CalibrationSuccess = false;
		m_CaptureNext = false;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool CCTool::on_utility_button(
		obs_properties_t* properties,
		obs_property_t* button,
		void* data
	)
	{
		CCTool* tool = static_cast<CCTool*>(data);

		if(tool->remaining_captures() == 0)
		{
			if(tool->request_calibration())
			{
				obs_property_set_description(button, UTILITY_BTN_CAPTURE_TEXT);
				tool->reset();
				return true;
			}
		}
		else if(tool->remaining_captures() == 1)
		{
			obs_property_set_description(button, UTILITY_BTN_CALIBRATE_TEXT);
			tool->request_capture();
			return true;
		}
		else
		{
			tool->request_capture();
			return false;
		}

		return false;
	}

//---------------------------------------------------------------------------------------------------------------------

	void CCTool::request_capture()
	{
		m_CaptureNext = true;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool CCTool::request_calibration()
	{
		if(!parameters_valid())
			return false;

		m_CalibrateNext = true;
		return true;
	}

//---------------------------------------------------------------------------------------------------------------------

	bool CCTool::parameters_valid() const
	{
		return !m_ProfileName.empty()
			 && m_SquareSize != SQUARE_SIZE_DEFAULT;
	}

//---------------------------------------------------------------------------------------------------------------------

	uint32_t CCTool::remaining_captures() const
	{
		return std::max<int>(REQ_CALIBRATION_FRAMES - m_Calibrator.calibration_frames(), 0);
	}

//---------------------------------------------------------------------------------------------------------------------

	std::tuple<std::string, cv::Scalar> CCTool::generate_calibration_status() const
	{
		if(m_CalibrationSuccess)
			return {std::string("calibration successful!"), draw::YUV_GREEN};

		if(m_CalibrationFail)
			return {std::string("calibration failed!"), draw::YUV_RED};

		if(m_ProfileName.empty())
			return {std::string("invalid profile (empty)"), draw::YUV_RED};

		if(m_SquareSize == SQUARE_SIZE_DEFAULT)
			return {std::string("invalid square size (0mm)"), draw::YUV_RED};

		if(remaining_captures() > 0)
			return {std::string("more captures required"), draw::YUV_RED};

		if(remaining_captures() == 0)
			return {std::string("ready for calibration"), draw::YUV_GREEN};

		return {"unknown", draw::YUV_RED};
	}

//---------------------------------------------------------------------------------------------------------------------

	void CCTool::draw_calibration_hud(cv::UMat& frame) const
	{
		//TODO: Use C++20 fmt as soon as GCC supports it.

		const cv::Point text_offset(0, 50);
		cv::Point text_origin(5, 40);

		draw::text(
			frame,
			"Profile: " + m_ProfileName,
			text_origin,
			draw::YUV_MAGENTA
		);
		text_origin += text_offset;

		draw::text(
			frame,
			"Square Size: " + std::to_string(m_SquareSize) + "mm",
			text_origin,
			draw::YUV_MAGENTA
		);
		text_origin += text_offset;

		draw::text(
			frame,
			"Captures Remaining: " + std::to_string(remaining_captures()),
			text_origin,
			draw::YUV_MAGENTA
		);
		text_origin += text_offset;

		const auto& [status_text, status_color] = generate_calibration_status();
		draw::text(
			frame,
			"Status: " + status_text,
			text_origin,
			status_color
		);
		text_origin += text_offset;
	}

//---------------------------------------------------------------------------------------------------------------------

	void CCTool::filter(cv::UMat& frame)
	{
		if(m_CaptureNext)
		{
			if(m_ImageSize != frame.size())
				reset();

			m_Calibrator.feed(frame);
			m_CaptureNext = false;
		}

		if(m_CalibrateNext)
		{
			auto parameters = m_Calibrator.calibrate(m_SquareSize);

			if(save_profile(parameters, m_ProfileName))
				m_CalibrationSuccess = true;
			else
				m_CalibrationFail = true;

			m_CalibrateNext = false;
		}

		draw_calibration_hud(frame);
	}

//---------------------------------------------------------------------------------------------------------------------

	bool CCTool::validate() const
	{
		return m_Context != nullptr;
	}

//---------------------------------------------------------------------------------------------------------------------
}
