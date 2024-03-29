#!/bin/bash

# Parse arguments
config="Release"
while getopts "c:" args; do
    case $args in
        c)
            config=${OPTARG}
    esac
done

# Create main paths
scripts_path=$(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)
deps_path="${scripts_path}/../Dependencies"

# Create dependencies directory
if [ ! -d "$deps_path" ]; then
    mkdir "$deps_path"
fi

# Install OpenCV build dependencies
sudo apt install \
     git \
     build-essential \
     cmake \
     pkg-config \
     qtbase5-dev \
     libavcodec-dev \
     libavformat-dev \
     libswscale-dev \
     libv4l-dev \
     libxvidcore-dev \
     libx264-dev \
     libjpeg-dev \
     libpng-dev \
     libatlas-base-dev \
     libtbb2 \
     libtbb-dev \
     libdc1394-dev

# Clone OpenCV
opencv_version="4.8.0"
opencv_path="${deps_path}/opencv"
if [ ! -d "$opencv_path" ]; then
    cd "$deps_path"
    git clone -b "$opencv_version" https://github.com/opencv/opencv.git
fi

if [ ! -d "$opencv_path" ]; then
   echo "Error! Could not clone OpenCV." 
   exit
fi


# Create build folder
opencv_build_path="${opencv_path}/build"
if [ ! -d "$opencv_build_path" ]; then
    mkdir "$opencv_build_path"
fi
cd "$opencv_build_path"

# Run CMake configuration
cmake -DCMAKE_BUILD_TYPE="$config" \
      -DBUILD_SHARED_LIBS=OFF \
      -DENABLE_LTO=ON \
      -DCV_TRACE=OFF \
      -DWITH_OPENCL=ON \
      -DWITH_OPENGL=ON \
      -DWITH_QT=ON \
      -DWITH_EIGEN=OFF \
      -DBUILD_EXAMPLES=OFF \
      -DBUILD_TESTS=OFF \
      -DBUILD_opencv_apps=OFF \
      -DBUILD_opencv_ts=OFF \
      -DBUILD_opencv_stitching=OFF \
      -DBUILD_opencv_objdetect=OFF \
      -DBUILD_opencv_python3=OFF \
      -DBUILD_opencv_photo=ON \
      -DBUILD_opencv_gapi=OFF \
      -DBUILD_opencv_dnn=OFF \
      -DBUILD_opencv_ml=OFF ..

# Compile and install OpenCV
cmake --build . --config "$config"
cmake --install . --prefix "./install/" --config "$config"


# Install OBS-Studio build dependencies (modified from wiki)
sudo apt install \
     libmbedtls-dev \
     libasound2-dev \
     libavcodec-dev \
     libavdevice-dev \
     libavfilter-dev \
     libavformat-dev \
     libavutil-dev \
     libcurl4-openssl-dev \
     libfdk-aac-dev \
     libfontconfig-dev \
     libfreetype6-dev \
     libglvnd-dev \
     libjack-jackd2-dev \
     libjansson-dev \
     libluajit-5.1-dev \
     libpulse-dev \
     libspeexdsp-dev \
     libswresample-dev \
     libswscale-dev \
     libudev-dev \
     libv4l-dev \
     libvlc-dev \
     libwayland-dev \
     libx11-dev \
     libx264-dev \
     libxcb-shm0-dev \
     libxcb-xinerama0-dev \
     libxcomposite-dev \
     libxinerama-dev \
     libxcb-randr0-dev \
     libxcb-xfixes0-dev \
     libx11-xcb-dev \
     libxcb1-dev \
     libxss-dev \
     libgles2-mesa \
     libgles2-mesa-dev \
     libpci-dev \
     libxkbcommon-dev \
     python3-dev \
     swig


# Clone Eigen
eigen_version="3.4"
eigen_path="${deps_path}/eigen"
if [ ! -d "$eigen_path" ]; then
    cd "$deps_path"
    git clone -b "$eigen_version" https://gitlab.com/libeigen/eigen.git
fi

if [ ! -d "$eigen_path" ]; then
    echo "Error! Could not clone Eigen." 
    exit
fi

# Create build folder
eigen_build_path="${eigen_path}/build"
if [ ! -d "$eigen_build_path" ]; then
    mkdir "$eigen_build_path"
fi
cd "$eigen_build_path"

# Build Eigen (this just generates the cmake package)
cmake ..
cmake --build .


# Clone OBS-Studio
obs_studio_version="27.2.4"
obs_studio_path="${deps_path}/obs-studio"
if [ ! -d "$obs_studio_path" ]; then
    cd "$deps_path"
    git clone -b "$obs_studio_version" --recursive https://github.com/obsproject/obs-studio.git
fi

if [ ! -d "$obs_studio_path" ]; then
    echo "Error! Could not clone OBS-Studio." 
    exit
fi


# Create build folder
obs_studio_build_path="${obs_studio_path}/build"
if [ ! -d "$obs_studio_build_path" ]; then
    mkdir "$obs_studio_build_path"
fi
cd "$obs_studio_build_path"

# Run CMake configuration
cmake -DCMAKE_BUILD_TYPE="$config" \
      -DDISABLE_UI=ON \
      -DENABLE_UI=OFF \
      -DBUILD_BROWSER=OFF \
      -DBUILD_VST=OFF \
      -DENABLE_SCRIPTING=OFF \
      -DENABLE_PIPEWIRE=OFF \
      ..

# Compile and install OBS-Studio
cmake --build . --config "$config"
cmake --install . --prefix "./install/" --config "$config"





