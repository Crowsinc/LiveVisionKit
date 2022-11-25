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

#include "FSRFilter.hpp"

#include <obs-module.h>

#include "Utility/OBSDispatch.hpp"
#include "Utility/Locale.hpp"

//---------------------------------------------------------------------------------------------------------------------

extern void register_fsr_source()
{
	obs_source_info config = {0};
	config.id = "LVK~FSR";
	config.type = OBS_SOURCE_TYPE_FILTER;
	config.output_flags = OBS_SOURCE_VIDEO | OBS_SOURCE_SRGB | OBS_SOURCE_CUSTOM_DRAW;

	config.create = lvk::dispatch::filter_create_auto<lvk::FSRFilter>;
	config.destroy = lvk::dispatch::filter_delete<lvk::FSRFilter>;

	config.update = lvk::dispatch::filter_configure<lvk::FSRFilter>;
	config.video_tick = lvk::dispatch::filter_tick<lvk::FSRFilter>;
	config.video_render = lvk::dispatch::filter_render<lvk::FSRFilter>;

	config.get_name = [](void* data){return L("fsr.name");};
	config.get_width = lvk::dispatch::filter_width<lvk::FSRFilter>;
	config.get_height = lvk::dispatch::filter_height<lvk::FSRFilter>;
	config.get_properties = lvk::dispatch::filter_properties<lvk::FSRFilter>;
	config.get_defaults = lvk::dispatch::filter_load_defaults<lvk::FSRFilter>;

	obs_register_source(&config);
}

//---------------------------------------------------------------------------------------------------------------------
