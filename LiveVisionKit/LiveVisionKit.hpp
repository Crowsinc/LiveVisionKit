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

// Library

#include <util/platform.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>

#include "Math/BoundingQuad.hpp"

#include "Math/Math.hpp"
#include "Math/Logic.hpp"
#include "Math/Homography.hpp"

#include "Structures/SlidingBuffer.hpp"

#include "Utility/Algorithm.hpp"
#include "Utility/Drawing.hpp"

#include "Vision/FrameTracker.hpp"
#include "Vision/GridDetector.hpp"
#include "Vision/CameraCalibrator.hpp"

#include "OBS/Utility/OBSDispatch.hpp"
#include "OBS/Utility/Logging.hpp"

#include "OBS/Effects/DefaultEffect.hpp"
#include "OBS/Effects/FSREffect.hpp"
#include "OBS/Effects/CASEffect.hpp"

#include "Diagnostics/Directives.hpp"
#include "OBS/Interop/VisionFilter.hpp"
#include "OBS/Interop/FrameBuffer.hpp"


// OBS Specific
#include <obs-module.h>
#include <util/config-file.h>
#include <util/platform.h>

