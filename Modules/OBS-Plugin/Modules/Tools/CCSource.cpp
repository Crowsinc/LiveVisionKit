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

#include <obs-module.h>

#include "OBS/Utility/OBSDispatch.hpp"
#include "OBS/Utility/Locale.hpp"

//---------------------------------------------------------------------------------------------------------------------

extern void register_cct_source()
{
	obs_source_info config = {0};
	config.id = "LVK~CCT";
	config.type = OBS_SOURCE_TYPE_FILTER;
	config.output_flags = OBS_SOURCE_ASYNC_VIDEO;

	config.create = lvk::dispatch::filter_create_auto<lvk::CCTool>;
	config.destroy = lvk::dispatch::filter_delete<lvk::CCTool>;

	config.update = lvk::dispatch::filter_configure<lvk::CCTool>;
	config.filter_video = lvk::dispatch::filter_process<lvk::CCTool>;

	config.get_name = [](void* data){return L("cct.name");};
	config.get_properties = lvk::dispatch::filter_properties<lvk::CCTool>;;
	config.get_defaults = lvk::dispatch::filter_load_defaults<lvk::CCTool>;;

	obs_register_source(&config);
}

//---------------------------------------------------------------------------------------------------------------------

extern void register_cct_effect_source()
{
	obs_source_info config = { 0 };
	config.id = "LVK~CCT~Effect";
	config.type = OBS_SOURCE_TYPE_FILTER;
	config.output_flags = OBS_SOURCE_VIDEO | OBS_SOURCE_CUSTOM_DRAW;

	config.create = lvk::dispatch::filter_create_auto<lvk::CCTool>;
	config.destroy = lvk::dispatch::filter_delete<lvk::CCTool>;

	config.update = lvk::dispatch::filter_configure<lvk::CCTool>;
	config.video_render = lvk::dispatch::filter_render<lvk::CCTool>;

	config.get_name = [](void* data) {return L("cct.name"); };
	config.get_properties = lvk::dispatch::filter_properties<lvk::CCTool>;;
	config.get_defaults = lvk::dispatch::filter_load_defaults<lvk::CCTool>;;

	obs_register_source(&config);
}

//---------------------------------------------------------------------------------------------------------------------
