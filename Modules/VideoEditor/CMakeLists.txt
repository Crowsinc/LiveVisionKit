
# Set up project 
project(lvk-editor CXX)
set(CMAKE_CXX_STANDARD 20)

# Set up executable 
add_executable(${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX ${LVK_DEBUG_POSTFIX})

set_property(TARGET ${PROJECT_NAME} PROPERTY PROJECT_LABEL "Video Editor")
set_property(TARGET ${PROJECT_NAME} PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
set_property(TARGET ${PROJECT_NAME} PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# Disable assert checks
if(DISABLE_CHECKS)
    add_definitions(-DLVK_DISABLE_CHECKS)
    add_definitions(-DNDEBUG)
endif()

# Project settings
message(STATUS "${MI}No Configuration Options.")

# Include all dependencies
target_include_directories(
    ${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR} 
        ${OpenCV_INCLUDE_DIRS}
        ${LVK_CORE_DIR}
)

# Link all dependencies
add_dependencies(${PROJECT_NAME} lvk-core)
target_link_libraries(
    ${PROJECT_NAME}
    lvk-core
    opencv_videoio
)


# Set up install rules
install(
    TARGETS ${PROJECT_NAME}
    DESTINATION ${LVK_RELEASES_DIR}
)

# Add executable sources
target_sources(
    ${PROJECT_NAME}
    PRIVATE
        Application.cpp
        VideoProcessor.hpp
        VideoProcessor.cpp
        VideoIOConfiguration.cpp
        VideoIOConfiguration.hpp
        ConsoleLogger.hpp
        ConsoleLogger.cpp
        OptionParser.hpp
        OptionParser.tpp
        FilterParser.hpp
        FilterParser.tpp
)

