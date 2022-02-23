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

#include "LCFilter.hpp"

#include <filesystem>

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------


	constexpr auto REQ_CALIBRATION_FRAMES = 50;

	constexpr auto PROP_CORRECT_DISTORTION = "PROP_DISTORTION";
	constexpr auto CORRECT_DISTORTION_DEFAULT = false;

	constexpr auto PROP_PROFILE_PATH = "PROP_PROFILE_PATH";
	constexpr auto PROP_NEW_PROFILE_NAME = "PROP_PROFILE_NAME";

	constexpr auto PROFILE_NAME_INVALID_CHARS = "><:\"\\/|?*";
	constexpr auto PROFILE_FILE_TYPE = "*.lvkc";

	constexpr auto GROUP_CALIBRATION = "GROUP_CALIBRATION";

	constexpr auto PROP_CALIB_BTN = "PROP_CALIBRATE_BTN";
	constexpr auto CALIB_BTN_CAP_MODE_TEXT = "Capture Frame";
	constexpr auto CALIB_BTN_CALIB_MODE_TEXT = "Calibrate";

//---------------------------------------------------------------------------------------------------------------------

	obs_properties_t* LCFilter::Properties()
	{
		// Filter properties
		obs_properties_t* properties = obs_properties_create();

		obs_properties_add_path(
			properties,
			PROP_PROFILE_PATH,
			"Calibration Profile",
			obs_path_type::OBS_PATH_FILE,
			PROFILE_FILE_TYPE,
			nullptr
		);

		obs_properties_add_bool(
			properties,
			PROP_CORRECT_DISTORTION,
			"Correct lens distortion"
		);

		// Calibration properties
		obs_properties_t* calib_properties = obs_properties_create();

		obs_properties_add_group(
			properties,
			GROUP_CALIBRATION,
			"Start Calibration",
			obs_group_type::OBS_GROUP_CHECKABLE,
			calib_properties
		);

		obs_properties_add_text(
			calib_properties,
			PROP_NEW_PROFILE_NAME,
			"New Profile Name",
			obs_text_type::OBS_TEXT_DEFAULT
		);

		auto property = obs_properties_add_button(
			calib_properties,
			PROP_CALIB_BTN,
			CALIB_BTN_CAP_MODE_TEXT,
			LCFilter::advance_calibration
		);
		obs_property_button_set_type(property, obs_button_type::OBS_BUTTON_DEFAULT);

		return properties;
	}

//---------------------------------------------------------------------------------------------------------------------

	void LCFilter::LoadDefaults(obs_data_t* settings)
	{
		obs_data_set_bool(settings, PROP_CORRECT_DISTORTION, CORRECT_DISTORTION_DEFAULT);
	}

//---------------------------------------------------------------------------------------------------------------------

	bool LCFilter::advance_calibration(
		obs_properties_t* properties,
		obs_property_t* button,
		void* data
	)
	{
		auto filter = static_cast<LCFilter*>(data);


		if(filter->captures_left() == 0)
		{
			obs_data_t* settings = obs_source_get_settings(filter->m_Context);

			// Block calibration until profile name is valid
			const std::string profile_name = obs_data_get_string(settings, PROP_NEW_PROFILE_NAME);
			if(is_profile_name_valid(profile_name))
			{
				auto path = filter->calibrate(profile_name);

				// If the profile field is empty, automatically set the profile to this new one
				if(std::string(obs_data_get_string(settings, PROP_PROFILE_PATH)).empty())
					obs_data_set_string(settings, PROP_PROFILE_PATH, path.c_str());


				// Switch back to calibration mode, and disable the group
				obs_property_set_description(button, CALIB_BTN_CAP_MODE_TEXT);
				obs_data_set_string(settings, PROP_NEW_PROFILE_NAME, "");
				obs_data_set_bool(settings, GROUP_CALIBRATION, false);
				return true;
			}
		}
		else if(filter->captures_left() == 1)
		{
			// Capture last frame and switch to calibration button
			obs_property_set_description(button, CALIB_BTN_CALIB_MODE_TEXT);
			filter->capture_next();
			return true;
		}
		else filter->capture_next();

		return false;
	}


//---------------------------------------------------------------------------------------------------------------------

	bool LCFilter::is_profile_name_valid(const std::string& profile_name)
	{
		return !profile_name.empty()
			&& profile_name.find_first_of(PROFILE_NAME_INVALID_CHARS) == std::string::npos;
	}

//---------------------------------------------------------------------------------------------------------------------

	LCFilter::LCFilter(obs_source_t* context)
		: VisionFilter(context),
		  m_Context(context),
		  m_CalibrationMode(false),
		  m_CaptureNext(false),
		  m_Captures(REQ_CALIBRATION_FRAMES)
	{
		LVK_ASSERT(context != nullptr);
	}

//---------------------------------------------------------------------------------------------------------------------


	void LCFilter::configure(obs_data_t* settings)
	{
		m_CalibrationMode = obs_data_get_bool(settings, GROUP_CALIBRATION);


		//TODO: ... load profile up if it has changed


	}

//---------------------------------------------------------------------------------------------------------------------

	void LCFilter::capture_next()
	{
		m_CaptureNext = true;
	}

//---------------------------------------------------------------------------------------------------------------------

	void LCFilter::reset_captures()
	{
		m_Captures.clear();
	}

//---------------------------------------------------------------------------------------------------------------------

	uint32_t LCFilter::captures_left()
	{
		return m_Captures.window_size() - m_Captures.elements();
	}

//---------------------------------------------------------------------------------------------------------------------

	std::filesystem::path LCFilter::calibrate(const std::string& profile_name)
	{
		LVK_ASSERT(is_profile_name_valid(profile_name));

		char* config_path = obs_module_config_path(profile_name.c_str());

		std::string full_path(config_path);

		blog(LOG_INFO, config_path);

		bfree(config_path);


		return {full_path};
	}

//---------------------------------------------------------------------------------------------------------------------

	void LCFilter::filter(cv::UMat& frame)
	{
		if(m_CalibrationMode)
		{
			obs_data_t* calibration_settings = obs_source_get_settings(m_Context);

			const std::string profile_name = obs_data_get_string(
				calibration_settings,
				PROP_NEW_PROFILE_NAME
			);

			const bool name_valid = is_profile_name_valid(profile_name);

			//TODO: switch to C++20 fmt as soon as GCC supports it.
			std::stringstream calibration_text;
			calibration_text << "Calibration Progress of " << m_Captures.elements() << "/" << REQ_CALIBRATION_FRAMES;
			calibration_text << " for \'" << profile_name << "\'" << (name_valid ? "" : "  (INVALID NAME)");

			draw::text(
				frame,
				calibration_text.str(),
				cv::Point(5, 45),
				name_valid ? draw::YUV_MAGENTA : draw::YUV_RED
			);

			if(m_CaptureNext)
				frame.copyTo(m_Captures.advance());

			m_CaptureNext = false;
		}


	}

//---------------------------------------------------------------------------------------------------------------------

	bool LCFilter::validate() const
	{
		return m_Context != nullptr;
	}

//---------------------------------------------------------------------------------------------------------------------

}
