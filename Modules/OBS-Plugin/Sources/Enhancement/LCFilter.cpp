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

#include "Utility/ScopedProfiler.hpp"
#include "Sources/Tools/CCTool.hpp"
#include "Utility/Locale.hpp"

#include <LiveVisionKit.hpp>

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	constexpr auto PROP_PROFILE = "PROP_PROFILE";
	constexpr auto PROFILE_DEFAULT = "";

	constexpr auto PROP_TEST_MODE = "PROP_TEST_MODE";
	constexpr auto TEST_MODE_DEFAULT = false;
    const cv::Size TEST_MODE_GRID = {32, 32};



//---------------------------------------------------------------------------------------------------------------------

	obs_properties_t* LCFilter::Properties()
	{
		obs_properties_t* properties = obs_properties_create();

		auto property = obs_properties_add_list(
			properties,
			PROP_PROFILE,
			L("f.calib-profile"),
			obs_combo_type::OBS_COMBO_TYPE_LIST,
			obs_combo_format::OBS_COMBO_FORMAT_STRING
		);

		obs_property_list_add_string(property, PROFILE_DEFAULT, PROFILE_DEFAULT);
		const auto& profiles = CCTool::ListProfiles();
		for(const auto& profile : profiles)
			obs_property_list_add_string(property, profile.c_str(), profile.c_str());


        obs_properties_add_bool(
            properties,
            PROP_TEST_MODE,
            L("f.testmode")
        );

		return properties;
	}

//---------------------------------------------------------------------------------------------------------------------

	void LCFilter::LoadDefaults(obs_data_t* settings)
	{
		obs_data_set_default_string(settings, PROP_PROFILE, PROFILE_DEFAULT);
        obs_data_set_default_bool(settings, PROP_TEST_MODE, TEST_MODE_DEFAULT);
	}

//---------------------------------------------------------------------------------------------------------------------

	LCFilter::LCFilter(obs_source_t* context)
		: VisionFilter(context),
		  m_Context(context)
	{
		LVK_ASSERT(context != nullptr);
	}

//---------------------------------------------------------------------------------------------------------------------

	void LCFilter::configure(obs_data_t* settings)
	{
		const std::string profile = obs_data_get_string(settings, PROP_PROFILE);
        m_ProfileSelected = profile != PROFILE_DEFAULT;

		if(m_ProfileSelected && m_Profile != profile)
		{
			if(auto parameters = CCTool::LoadProfile(profile); parameters.has_value())
			{
				m_Parameters = *parameters;
				m_Profile = profile;

				// Reset the undistort field to load in new profiles
                m_FieldOutdated = true;
			}
			else m_ProfileSelected = false;
		}

        m_TestMode = obs_data_get_bool(settings, PROP_TEST_MODE);
	}

//---------------------------------------------------------------------------------------------------------------------

	void LCFilter::prepare_undistort_maps(cv::UMat& frame)
	{
		// Update undistort map if it is outdated or missing
		if(m_FieldOutdated || m_CorrectionField.size() != frame.size())
		{
            cv::Rect view_region;
			cv::Mat optimal_camera_matrix = cv::getOptimalNewCameraMatrix(
				m_Parameters.camera_matrix,
				m_Parameters.distortion_coefficients,
				frame.size(),
				0,
				frame.size(),
				&view_region
			);

            cv::Mat correction_map;
			cv::initUndistortRectifyMap(
				m_Parameters.camera_matrix,
				m_Parameters.distortion_coefficients,
				cv::noArray(),
				optimal_camera_matrix,
				frame.size(),
				CV_32FC2,
                correction_map,
				cv::noArray()
			);

            // Convert the correction map to a warp field.
            m_CorrectionField.set_to(std::move(correction_map), false);
            m_CorrectionField.crop_in(view_region, frame.size());
            m_FieldOutdated = false;
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void LCFilter::filter(cv::UMat& frame)
	{
        LVK_PROFILE;

        if(m_TestMode)
        {
            // Draw grid so that the correction warp is visible.
            lvk::draw_grid(frame, TEST_MODE_GRID, lvk::yuv::MAGENTA, 3);
        }

		if(m_ProfileSelected)
		{
			prepare_undistort_maps(frame);

            m_CorrectionField.apply(frame, m_CorrectedFrame, true);
            std::swap(frame, m_CorrectedFrame);
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	bool LCFilter::validate() const
	{
		return m_Context != nullptr;
	}

//---------------------------------------------------------------------------------------------------------------------

}
