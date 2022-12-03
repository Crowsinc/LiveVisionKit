
# PROJECT DEFINITION
project(lvk-obs VERSION 1.3.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)

add_library(${PROJECT_NAME} SHARED)
set_property(TARGET ${PROJECT_NAME} PROPERTY PROJECT_LABEL "LVK OBS-Plugin")

add_definitions(-DVERSION="${PROJECT_VERSION}")
message(STATUS "${MI}Plugin Version: ${PROJECT_VERSION}")


# SETTINGS
option(
    OBS_PLUGIN_SHOW_LOCALE_TAG
    "Show locale tags for translations"
    "OFF"
)
if(OBS_PLUGIN_SHOW_LOCALE_TAG)
    add_definitions(-DLOCALE_SHOW_TAG)
endif()
message(STATUS "${MI}Show Locale Tags: ${OBS_PLUGIN_SHOW_LOCALE_TAG}")

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


# DEPENDENCIES
set(OBS_BUILD_PATH "" CACHE PATH "The path to the OBS-Studio build folder")

set(OBS_REL_INCLUDE_PATH "/../libobs/")
if(WIN32)
    set(OBS_REL_LIB_PATH "/install/bin/64bit/obs.lib")
else()
    set(OBS_REL_LIB_PATH "/install/bin/obs")
endif()

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

target_include_directories(
    ${PROJECT_NAME}
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR} 
    ${LVK_CORE_DIR}
    ${OBS_BUILD_PATH}${OBS_REL_INCLUDE_PATH}
)

add_dependencies(${PROJECT_NAME} lvk-core)
target_link_libraries(
    ${PROJECT_NAME}
    ${OBS_BUILD_PATH}${OBS_REL_LIB_PATH}
    lvk-core
)


# AUTO RELEASE & INSTALL
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
message(STATUS "${MI}${MI} (DEBUG) ${CMAKE_INSTALL_PREFIX}${PLUGIN_DEBUG_RELEASE_ROOT_DIR}")
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
            IMPORTED_RUNTIME_ARTIFACTS "${OpenCV_LIBS}"
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
        RUNTIME DESTINATION "${PLUGIN_DEBUG_RELEASE_ROOT_DIR}${PLUGIN_RELEASE_BIN_DIR}"
    )
    install(
        IMPORTED_RUNTIME_ARTIFACTS "${OpenCV_LIBS}"
        CONFIGURATIONS Debug
        DESTINATION "${PLUGIN_DEBUG_RELEASE_ROOT_DIR}${PLUGIN_RELEASE_BIN_DIR}"
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


# MODULE SOURCES
target_sources(
    ${PROJECT_NAME}
    PRIVATE
        Modules/Module.cpp
        Modules/Enhancement/ADBFilter.cpp
        Modules/Enhancement/ADBFilter.hpp
        Modules/Enhancement/ADBSource.cpp
        Modules/Enhancement/CASFilter.cpp
        Modules/Enhancement/CASFilter.hpp
        Modules/Enhancement/CASSource.cpp
        Modules/Enhancement/LCFilter.cpp
        Modules/Enhancement/LCFilter.hpp
        Modules/Enhancement/LCSource.cpp
        Modules/Scaling/FSRFilter.cpp
        Modules/Scaling/FSRFilter.hpp
        Modules/Scaling/FSRSource.cpp
        Modules/Stabilisation/VSFilter.cpp
        Modules/Stabilisation/VSFilter.hpp
        Modules/Stabilisation/VSSource.cpp
        Modules/Tools/CCSource.cpp
        Modules/Tools/CCTool.cpp
        Modules/Tools/CCTool.hpp
)

# EFFECT SOURCES
target_sources(
    ${PROJECT_NAME}
    PRIVATE
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
)

# INTEROP SOURCES
target_sources(
    ${PROJECT_NAME}
    PRIVATE
        Interop/FrameBuffer.cpp
        Interop/FrameBuffer.hpp
        Interop/FrameIngest.cpp
        Interop/FrameIngest.hpp
        Interop/InteropContext.cpp
        Interop/InteropContext.hpp
        Interop/VisionFilter.cpp
        Interop/VisionFilter.hpp
)

# UTILITY SOURCES
target_sources(
    ${PROJECT_NAME}
    PRIVATE
        Utility/Graphics.cpp
        Utility/Graphics.hpp
        Utility/Locale.hpp
        Utility/Logging.hpp
        Utility/Logging.tpp
        Utility/OBSDispatch.hpp
        Utility/OBSDispatch.tpp
)