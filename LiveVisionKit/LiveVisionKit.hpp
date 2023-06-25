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

#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>

#include "Directives.hpp"

#include "Functions/Math.hpp"
#include "Functions/Text.hpp"
#include "Functions/Image.hpp"
#include "Functions/Logic.hpp"
#include "Functions/Drawing.hpp"
#include "Functions/Container.hpp"
#include "Functions/Extensions.hpp"


#include "Filters/VideoFilter.hpp"
#include "Filters/CompositeFilter.hpp"
#include "Filters/ConversionFilter.hpp"
#include "Filters/DeblockingFilter.hpp"
#include "Filters/StabilizationFilter.hpp"

#include "Logging/Logger.hpp"
#include "Logging/CSVLogger.hpp"

#include "Math/WarpField.hpp"
#include "Math/Homography.hpp"
#include "Math/VirtualGrid.hpp"
#include "Math/BoundingQuad.hpp"

#include "Data/VideoFrame.hpp"
#include "Data/SpatialMap.hpp"
#include "Data/StreamBuffer.hpp"

#include "Timing/Time.hpp"
#include "Timing/Stopwatch.hpp"
#include "Timing/TickTimer.hpp"

#include "Utility/Unique.hpp"
#include "Utility/Configurable.hpp"

#include "Vision/FrameTracker.hpp"
#include "Vision/PathSmoother.hpp"
#include "Vision/FeatureDetector.hpp"
#include "Vision/CameraCalibrator.hpp"



