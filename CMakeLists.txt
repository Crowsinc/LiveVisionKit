cmake_minimum_required(VERSION 3.21)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMakeTargets")

if(APPLE)
    message(FATAL_ERROR "Apple systems are not supported!")
endif()


# PROJECT DEFINITION
project(LiveVisionKit)


# SETTINGS
message(STATUS "\nConfiguring LiveVisionKit...")

set(MI "   ")
set(LVK_DEBUG_POSTFIX "d")
set(LVK_RELEASES_DIR "Releases")
set(LVK_INCLUDES_DIR "Include")
set(LVK_BINARY_DIR "Binaries")

set(LVK_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/LiveVisionKit/")
set(DISABLE_CHECKS "OFF" CACHE BOOL "Compile without asserts and pre-condition checks")
set(BUILD_OBS_PLUGIN "ON" CACHE BOOL "Build the OBS plugin")
set(BUILD_VIDEO_PROCESSOR_CLT "ON" CACHE BOOL "Build the Video Processor Command-Line Tool")

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(
        CMAKE_INSTALL_PREFIX
        "${CMAKE_BINARY_DIR}/Install/"
        CACHE PATH
        "The location to which build files will be installed" 
        FORCE
    )
endif(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

if(WIN32)
    set(OPENCV_BUILD_PATH "" CACHE PATH "The path to the OpenCV build folder")
    set(OPENCV_MODULES world CACHE STRING "The desired OpenCV modules to link")
else()
    set(OPENCV_BUILD_PATH "" CACHE PATH "An optional path to the desired OpenCV build folder")
    set(
        OPENCV_MODULES 
        core;calib3d;features2d;imgproc;videoio;video;highgui;
        CACHE STRING 
        "The OpenCV modules to link"
    )
endif()


# COMMON DEPENDENCIES
if(OPENCV_BUILD_PATH STREQUAL "")
    if(WIN32)
        message(FATAL_ERROR "OPENCV_BUILD_PATH was not provided!")
    else()
        find_package(OpenCV REQUIRED ${OPENCV_MODULES})
    endif()
else()
    find_package(OpenCV REQUIRED ${OPENCV_MODULES} PATHS ${OPENCV_BUILD_PATH})
endif()

if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "Failed to find the required OpenCV libaries!")
else()
    message(STATUS "${MI}Found OpenCV: YES")
    message(STATUS "${MI}OpenCV Version: ${OpenCV_VERSION}")
    message(STATUS "${MI}OpenCV Modules: ${OpenCV_LIBS}")
endif()


# MODULES
add_subdirectory(LiveVisionKit)
if(BUILD_VIDEO_PROCESSOR_CLT)
    message(STATUS "\nBuilding with LVK Video Processor CLT...")
    add_subdirectory(Modules/LVK-Processor)
endif()
if(BUILD_OBS_PLUGIN)
    message(STATUS "\nBuilding with LVK OBS-Studio plugin...")
    add_subdirectory(Modules/OBS-Plugin)
endif()

message(STATUS "\n")