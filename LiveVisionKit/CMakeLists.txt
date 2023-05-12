
# PROJECT DEFINITION
project(lvk-core LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)

add_library(${PROJECT_NAME})
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
        Directives.hpp
        Directives.cpp
)


# FUNCTION SOURCES
target_sources(
    ${PROJECT_NAME}
    PRIVATE
        Functions/OpenCL/Kernels.hpp
        Functions/OpenCL/Kernels.cpp
        Functions/Extensions.hpp
        Functions/Extensions.cpp
        Functions/Container.hpp
        Functions/Container.tpp
        Functions/Drawing.hpp
        Functions/Drawing.tpp
        Functions/Image.hpp
        Functions/Image.cpp
        Functions/Logic.hpp
        Functions/Logic.tpp
        Functions/Math.hpp
        Functions/Math.tpp
        Functions/Text.hpp
        Functions/Text.tpp
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
        Filters/ScalingFilter.cpp
        Filters/ScalingFilter.hpp
        Filters/VideoFilter.cpp
        Filters/VideoFilter.hpp
        Filters/VideoFrame.cpp
        Filters/VideoFrame.hpp
)

# Logging SOURCES
target_sources(
        ${PROJECT_NAME}
        PRIVATE
        Logging/CSVLogger.cpp
        Logging/CSVLogger.hpp
        Logging/Logger.hpp
        Logging/Logger.tpp
)

# MATH SOURCES
target_sources(
    ${PROJECT_NAME}
    PRIVATE
        Math/BoundingQuad.cpp
        Math/BoundingQuad.hpp
        Math/Homography.cpp
        Math/Homography.hpp
        Math/WarpField.hpp
        Math/WarpField.cpp
        Math/VirtualGrid.hpp
        Math/VirtualGrid.cpp
)

# STRUCTURE SOURCES
target_sources(
    ${PROJECT_NAME}
    PRIVATE
        Structures/StreamBuffer.hpp
        Structures/StreamBuffer.tpp
        Structures/SpatialMap.hpp
        Structures/SpatialMap.tpp
        Structures/Iterators.hpp
        Structures/Iterators.tpp
)


# TIMING SOURCES
target_sources(
        ${PROJECT_NAME}
        PRIVATE
        Timing/Stopwatch.cpp
        Timing/Stopwatch.hpp
        Timing/TickTimer.cpp
        Timing/TickTimer.hpp
        Timing/Time.cpp
        Timing/Time.hpp
)


# UTILITY SOURCES
target_sources(
    ${PROJECT_NAME}
    PRIVATE
        Utility/Configurable.hpp
        Utility/Configurable.tpp
        Utility/Unique.hpp
        Utility/Unique.tpp
)


# VISION SOURCES
target_sources(
    ${PROJECT_NAME}
    PRIVATE
        Vision/CameraCalibrator.cpp
        Vision/CameraCalibrator.hpp
        Vision/FrameTracker.cpp
        Vision/FrameTracker.hpp
        Vision/GridDetector.cpp
        Vision/GridDetector.hpp
        Vision/PathSmoother.cpp
        Vision/PathSmoother.hpp
)
