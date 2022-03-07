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

#include "LiveVisionKit.hpp"
#include "VSFilter.hpp"

//---------------------------------------------------------------------------------------------------------------------

constexpr auto VS_FILTER_NAME = "(LVK) Video Stabilizer";

//---------------------------------------------------------------------------------------------------------------------

extern void register_vs_source()
{
	obs_source_info config = {0};
	config.id = "LVK~VS";
	config.type = OBS_SOURCE_TYPE_FILTER;
	config.output_flags = OBS_SOURCE_ASYNC_VIDEO;

	config.create = lvk::dispatch::filter_create_auto<lvk::VSFilter>;
	config.destroy = lvk::dispatch::skip<void*>;
	config.filter_remove = lvk::dispatch::filter_delete<lvk::VSFilter, obs_source_t*>;

	config.update = lvk::dispatch::filter_configure<lvk::VSFilter>;
	config.video_tick = lvk::dispatch::filter_tick<lvk::VSFilter>;
	config.video_render = lvk::dispatch::filter_render<lvk::VSFilter>;
	config.filter_video = lvk::dispatch::filter_process<lvk::VSFilter>;

	config.get_name = [](void* data){return VS_FILTER_NAME;};
	config.get_width = lvk::dispatch::filter_width<lvk::VSFilter>;
	config.get_height = lvk::dispatch::filter_height<lvk::VSFilter>;
	config.get_properties = lvk::dispatch::filter_properties<lvk::VSFilter>;
	config.get_defaults = lvk::dispatch::filter_load_defaults<lvk::VSFilter>;

	obs_register_source(&config);
}

//---------------------------------------------------------------------------------------------------------------------
