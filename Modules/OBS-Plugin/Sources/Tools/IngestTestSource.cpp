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

#include "IngestTestFilter.hpp"

#include <obs-module.h>

#include "Utility/OBSDispatch.hpp"
#include "Utility/Locale.hpp"

//---------------------------------------------------------------------------------------------------------------------

extern void register_ingest_test_source()
{
	obs_source_info config = {nullptr};
	config.id = "LVK~INJ_TEST";
	config.type = OBS_SOURCE_TYPE_FILTER;
	config.output_flags = OBS_SOURCE_ASYNC_VIDEO;

	config.create = lvk::dispatch::filter_create<lvk::IngestTestFilter>;
	config.destroy = lvk::dispatch::filter_delete<lvk::IngestTestFilter>;
	config.get_name = [](void* data){return "(LVK) Ingest Test";};

	config.filter_video = lvk::dispatch::filter_process<lvk::IngestTestFilter>;

	obs_register_source(&config);
}

//---------------------------------------------------------------------------------------------------------------------