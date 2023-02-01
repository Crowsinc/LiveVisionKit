
# PROJECT DEFINITION
project(lvk-core LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)

add_library(${PROJECT_NAME} Structures/SpatialMap.hpp Structures/SpatialMap.tpp Vision/Tracking/WarpField.cpp Vision/Tracking/WarpField.hpp)
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX ${LVK_DEBUG_POSTFIX})
set_property(TARGET ${PROJECT_NAME} PROPERTY PROJECT_LABEL "LiveVisionKit")
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

if(DISABLE_CHECKS)
    add_definitions(-DLVK_DISABLE_CHECKS)
    add_definitions(-DNDEBUG)
endif()


# DEPENDENCIES
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

target_include_directories(
    ${PROJECT_NAME} 
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR} 
        ${OpenCV_INCLUDE_DIRS}
)    
target_link_libraries(
    ${PROJECT_NAME} 
    ${OpenCV_LIBS}
    Threads::Threads
)


# INSTALL 
install(
    TARGETS ${PROJECT_NAME}
    DESTINATION ${LVK_BINARY_DIR}
)
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    DESTINATION ${LVK_INCLUDES_DIR}
    FILES_MATCHING  
        PATTERN "*.hpp"
        PATTERN "*.tpp"
        PATTERN "*.h"
)


# CORE SOURCES
target_sources(
    ${PROJECT_NAME}
    PRIVATE
        LiveVisionKit.hpp
)

# DIAGNOSTICS SOURCES
target_sources(
    ${PROJECT_NAME}
    PRIVATE
        Diagnostics/Directives.hpp
        Diagnostics/Directives.cpp
        Diagnostics/Logging/CSVLogger.cpp
        Diagnostics/Logging/CSVLogger.hpp
        Diagnostics/Logging/Logger.hpp
        Diagnostics/Logging/Logger.tpp
)

# FILTER SOURCES
target_sources(
    ${PROJECT_NAME}
    PRIVATE
        Filters/CompositeFilter.cpp
        Filters/CompositeFilter.hpp
        Filters/ConversionFilter.cpp
        Filters/ConversionFilter.hpp
        Filters/DeblockingFilter.cpp
        Filters/DeblockingFilter.hpp
        Filters/StabilizationFilter.cpp
        Filters/StabilizationFilter.hpp
        Filters/VideoFilter.cpp
        Filters/VideoFilter.hpp
        Filters/VideoFrame.cpp
        Filters/VideoFrame.hpp
)

# MATH SOURCES
target_sources(
    ${PROJECT_NAME}
    PRIVATE
        Math/BoundingQuad.cpp
        Math/BoundingQuad.hpp
        Math/Homography.cpp
        Math/Homography.hpp
        Math/Logic.hpp
        Math/Logic.tpp
        Math/Math.hpp
        Math/Math.tpp
)

# STRUCTURE SOURCES
target_sources(
    ${PROJECT_NAME}
    PRIVATE
        Structures/SlidingBuffer.hpp
        Structures/SlidingBuffer.tpp
)

# UTILITY SOURCES
target_sources(
    ${PROJECT_NAME}
    PRIVATE
        Utility/Algorithm.hpp
        Utility/Algorithm.tpp
        Utility/Drawing.hpp
        Utility/Drawing.tpp
        Utility/Properties/Configurable.hpp
        Utility/Properties/Configurable.tpp
        Utility/Properties/Unique.hpp
        Utility/Properties/Unique.tpp
        Utility/Timing/Stopwatch.cpp
        Utility/Timing/Stopwatch.hpp
        Utility/Timing/TickTimer.cpp
        Utility/Timing/TickTimer.hpp
        Utility/Timing/Time.cpp
        Utility/Timing/Time.hpp
)

# VISION SOURCES
target_sources(
    ${PROJECT_NAME}
    PRIVATE
        Vision/Camera/CameraCalibrator.cpp
        Vision/Camera/CameraCalibrator.hpp
        Vision/Tracking/FrameTracker.cpp
        Vision/Tracking/FrameTracker.hpp
        Vision/Tracking/GridDetector.cpp
        Vision/Tracking/GridDetector.hpp
        Vision/Tracking/PathStabilizer.cpp
        Vision/Tracking/PathStabilizer.hpp
)
