
# Set up project 
project(lvk-obs VERSION 1.3.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)


# Set up library 
add_library(${PROJECT_NAME} SHARED)

set_property(TARGET ${PROJECT_NAME} PROPERTY PROJECT_LABEL "OBS-Plugin")
set_property(TARGET ${PROJECT_NAME} PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
set_property(TARGET ${PROJECT_NAME} PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# Disable assert checks
if(DISABLE_CHECKS)
    add_definitions(-DLVK_DISABLE_CHECKS)
    add_definitions(-DNDEBUG)
endif()

# Define the plugin version
add_definitions(-DVERSION="${PROJECT_VERSION}")
message(STATUS "${MI}Plugin Version: ${PROJECT_VERSION}")

# Project settings
option(OBS_PLUGIN_SHOW_LOCALE_TAG "Show locale tags for translations" "OFF")
if(OBS_PLUGIN_SHOW_LOCALE_TAG)
    add_definitions(-DLOCALE_SHOW_TAG)
endif()
message(STATUS "${MI}Show Locale Tags: ${OBS_PLUGIN_SHOW_LOCALE_TAG}")

# Install settings
option(
    OBS_PLUGIN_AUTO_INSTALL 
    "Automatically install the plugin into the given obs-studio folders (RELEASE ONLY)"
)
if(WIN32)
    set(
        OBS_PLUGIN_AUTO_INSTALL_PATHS 
        "C:/Program Files/obs-studio" 
        CACHE PATH 
        "The plugin auto-install paths, leading to the root obs-studio directory"
    )
else()
    set(
        OBS_PLUGIN_AUTO_INSTALL_PATHS 
        "~/.config/obs-studio" 
        CACHE PATH 
        "The plugin auto-install paths, leading to the root obs-studio directory"
    )
endif()


# Set the OBS-Studio build path 
set(OBS_BUILD_PATH "./Dependencies/obs-studio/build" CACHE PATH "The path to the OBS-Studio build folder")
if(WIN32)
    set(OBS_REL_INCLUDE_PATH "/install/include/")
    set(OBS_REL_LIB_PATH "/install/bin/64bit/obs.lib")
else()
    set(OBS_REL_INCLUDE_PATH "/install/include/obs/")
    set(OBS_REL_LIB_PATH "/install/lib/libobs.so")
endif()

# Check that the build path exists
if(OBS_BUILD_PATH STREQUAL "")
    message(FATAL_ERROR "OBS_BUILD_PATH was not provided!")
else()
    if(NOT EXISTS ${OBS_BUILD_PATH}${OBS_REL_INCLUDE_PATH})
        message(FATAL_ERROR "Could not find OBS-Studio includes - Check OBS_BUILD_PATH")
    endif()
    if(NOT EXISTS ${OBS_BUILD_PATH}${OBS_REL_LIB_PATH})
        message(FATAL_ERROR "Could not find OBS-Studio libraries - Check OBS_BUILD_PATH")
    endif()
    message(STATUS "${MI}Found OBS-Studio: YES")
endif()

# Include all dependencies
target_include_directories(
    ${PROJECT_NAME}
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR} 
    ${LVK_CORE_DIR}
    ${OBS_BUILD_PATH}${OBS_REL_INCLUDE_PATH}
)

# Link all dependencies
add_dependencies(${PROJECT_NAME} lvk-core)
target_link_libraries(
    ${PROJECT_NAME}
    ${OBS_BUILD_PATH}${OBS_REL_LIB_PATH}
    lvk-core
)


# Set up install rules
set(PLUGIN_RELEASE_NAME "LiveVisionKit-${PROJECT_VERSION}-${CMAKE_SYSTEM_NAME}")
set(PLUGIN_DEBUG_RELEASE_NAME "DEBUG_${PLUGIN_RELEASE_NAME}")

set(PLUGIN_RELEASE_ROOT_DIR "${LVK_RELEASES_DIR}/${PLUGIN_RELEASE_NAME}/obs-studio")
set(PLUGIN_DEBUG_RELEASE_ROOT_DIR "${LVK_RELEASES_DIR}/${PLUGIN_DEBUG_RELEASE_NAME}/obs-studio")

set(PLUGIN_RELEASE_INSTALL_DIRS "${CMAKE_INSTALL_PREFIX}/${PLUGIN_RELEASE_ROOT_DIR}")
if(OBS_PLUGIN_AUTO_INSTALL)
    list(APPEND PLUGIN_RELEASE_INSTALL_DIRS ${OBS_PLUGIN_AUTO_INSTALL_PATHS})
endif()

message(STATUS "${MI}Auto-Install Plugin: " ${OBS_PLUGIN_AUTO_INSTALL})
message(STATUS "${MI}Install Paths:" )
message(STATUS "${MI}${MI} (DEBUG) ${CMAKE_INSTALL_PREFIX}/${PLUGIN_DEBUG_RELEASE_ROOT_DIR}")
foreach(INSTALL_DIR ${PLUGIN_RELEASE_INSTALL_DIRS})
    message(STATUS "${MI}${MI} (RELEASE) ${INSTALL_DIR}")
endforeach()

if(WIN32)
    set(PLUGIN_RELEASE_BIN_DIR "/obs-plugins/64bit") 
    set(PLUGIN_RELEASE_DATA_DIR "/data/obs-plugins/${PROJECT_NAME}") 

    # Optimized
    foreach(INSTALL_DIR ${PLUGIN_RELEASE_INSTALL_DIRS})
        install(
            TARGETS ${PROJECT_NAME}
            CONFIGURATIONS Release MinSizeRel RelWithDebInfo
            RUNTIME DESTINATION "${INSTALL_DIR}${PLUGIN_RELEASE_BIN_DIR}"
        )
        install(
            DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Data/
            CONFIGURATIONS Release MinSizeRel RelWithDebInfo
            DESTINATION "${INSTALL_DIR}${PLUGIN_RELEASE_DATA_DIR}"
        )
    endforeach()
    
    # Debug
    install(
        TARGETS ${PROJECT_NAME}
        CONFIGURATIONS Debug
        RUNTIME DESTINATION "${PLUGIN_DEBUG_RELEASE_ROOT_DIR}${PLUGIN_RELEASE_BIN_DIR}"
    )
    install(
        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Data/
        CONFIGURATIONS Debug
        DESTINATION "${PLUGIN_DEBUG_RELEASE_ROOT_DIR}${PLUGIN_RELEASE_DATA_DIR}"
    )
else()
    set(PLUGIN_RELEASE_BIN_DIR "/plugins/${PROJECT_NAME}/bin/64bit") 
    set(PLUGIN_RELEASE_DATA_DIR "/plugins/${PROJECT_NAME}/data") 

    # Optimized
    foreach(INSTALL_DIR ${PLUGIN_RELEASE_INSTALL_DIRS})
        install(
            TARGETS ${PROJECT_NAME}
            CONFIGURATIONS Release MinSizeRel RelWithDebInfo
            DESTINATION "${INSTALL_DIR}${PLUGIN_RELEASE_BIN_DIR}"
        )
        install(
            DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Data/
            CONFIGURATIONS Release MinSizeRel RelWithDebInfo
            DESTINATION "${INSTALL_DIR}${PLUGIN_RELEASE_DATA_DIR}"
        )
    endforeach()

    # Debug
    install(
        TARGETS ${PROJECT_NAME}
        CONFIGURATIONS Debug
        DESTINATION "${PLUGIN_DEBUG_RELEASE_ROOT_DIR}${PLUGIN_RELEASE_BIN_DIR}"
    )
    install(
        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Data/
        CONFIGURATIONS Debug
        DESTINATION "${PLUGIN_DEBUG_RELEASE_ROOT_DIR}${PLUGIN_RELEASE_DATA_DIR}"
    )
endif()


# Add library sources
target_sources(
    ${PROJECT_NAME}
    PRIVATE
        Sources/Module.cpp
        Sources/Enhancement/ADBFilter.cpp
        Sources/Enhancement/ADBFilter.hpp
        Sources/Enhancement/ADBSource.cpp
        Sources/Enhancement/CASFilter.cpp
        Sources/Enhancement/CASFilter.hpp
        Sources/Enhancement/CASSource.cpp
        Sources/Enhancement/LCFilter.cpp
        Sources/Enhancement/LCFilter.hpp
        Sources/Enhancement/LCSource.cpp
        Sources/Scaling/FSRFilter.cpp
        Sources/Scaling/FSRFilter.hpp
        Sources/Scaling/FSRSource.cpp
        Sources/Stabilisation/VSFilter.cpp
        Sources/Stabilisation/VSFilter.hpp
        Sources/Stabilisation/VSSource.cpp
        Sources/Tools/CCSource.cpp
        Sources/Tools/CCTool.cpp
        Sources/Tools/CCTool.hpp

        Data/effects/ffx_a.h
        Data/effects/ffx_cas.h
        Data/effects/ffx_fsr1.h
        Effects/CASEffect.cpp
        Effects/CASEffect.hpp
        Effects/DefaultEffect.cpp
        Effects/DefaultEffect.hpp
        Effects/FSREffect.cpp
        Effects/FSREffect.hpp
        Effects/OBSEffect.hpp
        Effects/OBSEffect.tpp

        Interop/FrameBuffer.cpp
        Interop/FrameBuffer.hpp
        Interop/FrameIngest.cpp
        Interop/FrameIngest.hpp
        Interop/InteropContext.cpp
        Interop/InteropContext.hpp
        Interop/VisionFilter.cpp
        Interop/VisionFilter.hpp

        Utility/Graphics.cpp
        Utility/Graphics.hpp
        Utility/Locale.hpp
        Utility/Logging.hpp
        Utility/Logging.tpp
        Utility/OBSDispatch.hpp
        Utility/OBSDispatch.tpp
        Utility/ScopedProfiler.hpp
        Utility/ScopedProfiler.cpp
)
