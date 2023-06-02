#!/bin/bash

# Parse arguments
config="Release"
build_plugin=false
build_editor=false
install_plugin=false
while getopts "peic:" args; do
    case $args in
        c)
            config=${OPTARG}
            ;;
        p)
            build_plugin=true
            ;;
        e)
            build_editor=true
            ;;
        i)
            install_plugin=true
            ;;
    esac
done

# Call setup script
./setup_deb.sh -c "$config"

# Create main paths
scripts_path=$(pwd)
project_path="${scripts_path}/.."

# Create LVK build folder
build_path="${project_path}/Build"
if [ ! -d "$build_path" ]; then
    mkdir "$build_path"
fi
cd "$build_path"

# Resolve dependency build paths
opencv_build_path="${project_path}/Dependencies/opencv/build"
obs_studio_build_path="${project_path}/Dependencies/obs-studio/build"

# Run LVK CMake configuration
cmake '-DCMAKE_BUILD_TYPE=Debug;Release;RelWithDebInfo' \
      -DBUILD_OBS_PLUGIN="$build_plugin" \
      -DBUILD_VIDEO_EDITOR="$build_editor" \
      -DOBS_PLUGIN_AUTO_INSTALL="$install_plugin" \
      -DOBS_BUILD_PATH="$obs_studio_build_path" \
      -DOPENCV_BUILD_PATH="$opencv_build_path" \
      ..

# Compile LiveVisionKit and install it to the build folder.
cmake --build . --config "$config"
cmake --install . --config "$config"

