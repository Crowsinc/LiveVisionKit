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
#include "Utility/Logging.hpp"
#include "Utility/Locale.hpp"

#include <LiveVisionKit.hpp>

namespace lvk
{

//---------------------------------------------------------------------------------------------------------------------

	constexpr auto PROP_PROFILE = "PROP_PROFILE";
	constexpr auto PROP_PROFILE_DEFAULT = "";

	constexpr auto PROP_TEST_MODE = "PROP_TEST_MODE";
	constexpr auto PROP_TEST_MODE_DEFAULT = false;
    const cv::Size PROP_TEST_MODE_GRID = {32, 32};

//---------------------------------------------------------------------------------------------------------------------

	obs_properties_t* LCFilter::Properties()
	{
		obs_properties_t* properties = obs_properties_create();

        // Profile Selector
		auto property = obs_properties_add_list(
			properties,
			PROP_PROFILE,
			L("f.calib-profile"),
			obs_combo_type::OBS_COMBO_TYPE_LIST,
			obs_combo_format::OBS_COMBO_FORMAT_STRING
		);

		obs_property_list_add_string(property, PROP_PROFILE_DEFAULT, PROP_PROFILE_DEFAULT);
		const auto& profiles = CCTool::ListProfiles();
		for(const auto& profile : profiles)
			obs_property_list_add_string(property, profile.c_str(), profile.c_str());


        // Runtime Controls
        obs_properties_t* controls = obs_properties_create();
        obs_properties_add_group(
            properties,
            "CONTROL_GROUP",
            L("f.controls-group"),
            obs_group_type::OBS_GROUP_NORMAL,
            controls
        );

        // Test Mode Toggle
        obs_properties_add_bool(
            controls,
            PROP_TEST_MODE,
            L("f.testmode")
        );

		return properties;
	}

//---------------------------------------------------------------------------------------------------------------------

	void LCFilter::LoadDefaults(obs_data_t* settings)
	{
		obs_data_set_default_string(settings, PROP_PROFILE, PROP_PROFILE_DEFAULT);
        obs_data_set_default_bool(settings, PROP_TEST_MODE, PROP_TEST_MODE_DEFAULT);
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
        m_ProfileSelected = profile != PROP_PROFILE_DEFAULT;

		if(m_ProfileSelected && m_Profile != profile)
		{
			if(auto parameters = CCTool::LoadProfile(profile); parameters.has_value())
			{
				m_Parameters = *parameters;
				m_Profile = profile;

				// Reset the correction mesh to load in new profiles
                m_MeshOutdated = true;
			}
			else m_ProfileSelected = false;
		}

        m_TestMode = obs_data_get_bool(settings, PROP_TEST_MODE);

        // Print out settings
#ifdef PRINT_SETTINGS
        lvk::log::print_settings(
            m_Context,
            "\n    Profile: %s"
            "\n    Test Mode: %s",
            m_Profile.c_str(),
            m_TestMode ? "Yes" : "No"
        );
#endif
	}

//---------------------------------------------------------------------------------------------------------------------

	void LCFilter::prepare_undistort_maps(OBSFrame& frame)
	{
		// Update undistort map if it is outdated or missing
		if(m_MeshOutdated || m_CorrectionMesh.size() != frame.size())
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

            // Normalize the view region
            cv::Rect2f norm_view_region(
                cv::Point2f(view_region.tl()) / cv::Size2f(correction_map.size()),
                cv::Size2f(view_region.size()) / cv::Size2f(correction_map.size())
            );

            // Convert the correction map to a warp field.
            m_CorrectionMesh.set_to(std::move(correction_map), false, false);
            m_CorrectionMesh.crop_in(norm_view_region);
            m_MeshOutdated = false;
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void LCFilter::filter(OBSFrame& frame)
	{
        LVK_PROFILE;

        if(m_TestMode)
        {
            // Draw grid so that the correction warp is visible.
            lvk::draw_grid(frame, PROP_TEST_MODE_GRID, col::MAGENTA[frame.format], 1);
        }

		if(m_ProfileSelected)
		{
			prepare_undistort_maps(frame);

            m_CorrectionMesh.apply(frame, m_CorrectedFrame);
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
