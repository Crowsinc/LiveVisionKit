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

#include <LiveVisionKit.hpp>

#include "Interop/VisionFilter.hpp"

namespace lvk
{

	class LCFilter : public VisionFilter
	{
	public:

		static obs_properties_t* Properties();

		static void LoadDefaults(obs_data_t* settings);

		explicit LCFilter(obs_source_t* context);

		void configure(obs_data_t* settings);

		bool validate() const;

	private:

		void prepare_undistort_maps(OBSFrame& frame);

		void filter(OBSFrame& frame) override;

	private:
		obs_source_t* m_Context = nullptr;
		bool m_ProfileSelected = false;
        bool m_TestMode = false;

		std::string m_Profile;
		CameraParameters m_Parameters;

        bool m_FieldOutdated = true;
        WarpField m_CorrectionField{WarpField::MinimumSize};
        OBSFrame m_CorrectedFrame;
	};

}
