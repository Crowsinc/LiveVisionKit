
# PROJECT DEFINITION
project(lvk-processor CXX)
set(CMAKE_CXX_STANDARD 20)

set(CLT_NAME "lvk")
add_executable(${CLT_NAME})
set_target_properties(${CLT_NAME} PROPERTIES DEBUG_POSTFIX ${LVK_DEBUG_POSTFIX})
set_property(TARGET ${CLT_NAME} PROPERTY PROJECT_LABEL "LVK Video Processor")

message("${MI}No Configuration Options...")


# DEPENDENCIES
target_include_directories(
    ${CLT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR} 
        ${LVK_CORE_DIR}
)

add_dependencies(${CLT_NAME} lvk-core)
target_link_libraries(
    ${CLT_NAME}
    lvk-core
)


# INSTALL
install(
    TARGETS ${CLT_NAME}
    DESTINATION ${LVK_RELEASES_DIR}
)
if(WIN32)
    install(
        IMPORTED_RUNTIME_ARTIFACTS "${OpenCV_LIBS}"
        DESTINATION ${LVK_RELEASES_DIR}
    )
endif()


# CORE SOURCES
target_sources(
    ${CLT_NAME}
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

