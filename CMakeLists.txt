cmake_minimum_required(VERSION 3.21)

if(APPLE)
    message(FATAL_ERROR "Apple systems are not supported!")
endif()

set(MI "   ") # Message indents
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMakeTargets")


# Set up project 
project(LiveVisionKit)
message(STATUS "\nConfiguring LiveVisionKit...")

# Project defines
set(LVK_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/LiveVisionKit/")
set(LVK_RELEASES_DIR "Releases")
set(LVK_INCLUDES_DIR "Include")
set(LVK_BINARY_DIR "Binaries")
set(LVK_DEBUG_POSTFIX "d")

# Set default install location
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(
        CMAKE_INSTALL_PREFIX
        "${CMAKE_BINARY_DIR}/Install/"
        CACHE PATH
        "The location to which build files will be installed" 
        FORCE
    )
endif(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

# Project settings
set(BUILD_OBS_PLUGIN "ON" CACHE BOOL "Build the OBS-Studio plugin")
set(BUILD_VIDEO_EDITOR "ON" CACHE BOOL "Build the video editor CLT")
set(OPENCV_BUILD_PATH "" CACHE PATH "The path to the OpenCV build folder")
set(DISABLE_CHECKS "OFF" CACHE BOOL "Compile without asserts and pre-condition checks")

# Load common dependencies (OpenCV)
find_package(OpenCV REQUIRED PATHS ${OPENCV_BUILD_PATH})
if(OpenCV_FOUND)
    message(STATUS "${MI}Found OpenCV: YES")
    message(STATUS "${MI}OpenCV Version: ${OpenCV_VERSION}")
    message(STATUS "${MI}OpenCV Modules: ${OpenCV_LIBS}")
else()
    message(FATAL_ERROR "Failed to find OpenCV!")
endif()

# Load sub-modules
add_subdirectory(LiveVisionKit)

if(BUILD_VIDEO_EDITOR)                                 
    message(STATUS "\nBuilding with video editor CLT...")
    add_subdirectory(Modules/VideoEditor)
endif()

if(BUILD_OBS_PLUGIN)
    message(STATUS "\nBuilding with LVK OBS-Studio plugin...")
    add_subdirectory(Modules/OBS-Plugin)
endif()

message(STATUS "\n")