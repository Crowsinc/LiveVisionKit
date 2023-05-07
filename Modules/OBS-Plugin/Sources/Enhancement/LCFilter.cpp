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

	constexpr auto PROP_CORRECT_DISTORTION = "PROP_DISTORTION";
	constexpr auto CORRECT_DISTORTION_DEFAULT = false;

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
			PROP_CORRECT_DISTORTION,
			L("lc.correct-distortion")
		);

		return properties;
	}

//---------------------------------------------------------------------------------------------------------------------

	void LCFilter::LoadDefaults(obs_data_t* settings)
	{
		obs_data_set_default_bool(settings, PROP_CORRECT_DISTORTION, CORRECT_DISTORTION_DEFAULT);
		obs_data_set_default_string(settings, PROP_PROFILE, PROFILE_DEFAULT);
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
		bool profile_selected = profile != PROFILE_DEFAULT;

		if(profile_selected && m_Profile != profile)
		{
			if(auto parameters = CCTool::LoadProfile(profile); parameters.has_value())
			{
				m_Parameters = *parameters;
				m_Profile = profile;

				// Reset the undistort field to load in new profiles
                m_FieldOutdated = true;
			}
			else profile_selected = false;
		} 

		m_CorrectDistortion = profile_selected && obs_data_get_bool(settings, PROP_CORRECT_DISTORTION);
	}

//---------------------------------------------------------------------------------------------------------------------

	void LCFilter::prepare_undistort_maps(cv::UMat& frame)
	{
		// Update undistort map if it is outdated or missing
		if(m_FieldOutdated || m_UndistortField.size() != frame.size())
		{
            cv::Rect undistort_crop;
			cv::Mat optimal_camera_matrix = cv::getOptimalNewCameraMatrix(
				m_Parameters.camera_matrix,
				m_Parameters.distortion_coefficients,
				frame.size(),
				0,
				frame.size(),
				&undistort_crop
			);

            cv::Mat undistort_map;
			cv::initUndistortRectifyMap(
				m_Parameters.camera_matrix,
				m_Parameters.distortion_coefficients,
				cv::noArray(),
				optimal_camera_matrix,
				frame.size(),
				CV_32FC2,
                undistort_map,
				cv::noArray()
			);

            // Convert the undistort map to a warp field.
            m_UndistortField = WarpField(std::move(undistort_map), true);

            // Upscale the field to apply the crop in the same operation.
            // TODO: this won't be enough if the crop also applies an offset!
            m_UndistortField.scale(cv::Size2f(frame.size()) / cv::Size2f(undistort_crop.size()));

            m_FieldOutdated = false;
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	void LCFilter::filter(cv::UMat& frame)
	{
        LVK_PROFILE;

		if(m_CorrectDistortion)
		{
			prepare_undistort_maps(frame);

            m_UndistortField.warp(frame, m_UndistortFrame);
            std::swap(frame, m_UndistortFrame);
		}
	}

//---------------------------------------------------------------------------------------------------------------------

	bool LCFilter::validate() const
	{
		return m_Context != nullptr;
	}

//---------------------------------------------------------------------------------------------------------------------

}
