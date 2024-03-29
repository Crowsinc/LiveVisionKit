param(
    $config="Release"
)

# Create main paths
$scripts_path = $PSScriptRoot
$deps_path= Join-Path $scripts_path "/../Dependencies"

# Create dependencies directory
if(-Not (Test-Path $deps_path))
{
    New-Item -Path $deps_path -ItemType Directory
}

# Clone OpenCV
$opencv_version = "4.8.0"
$opencv_path = Join-Path $deps_path "/opencv"
if(-Not (Test-Path $opencv_path))
{
    Set-Location $deps_path
    git clone -b $opencv_version https://github.com/opencv/opencv.git
}

if(-Not (Test-Path $opencv_path))
{
    Write-Error "Error! Could not clone OpenCV."
    exit
}

# Create build folder
$opencv_build_path = Join-Path $opencv_path "/build"
if(-Not (Test-Path $opencv_build_path))
{
    New-Item -Path $opencv_build_path -ItemType Directory
}
Set-Location $opencv_build_path

# Run CMake configuration
cmake -DBUILD_SHARED_LIBS=OFF `
      -DENABLE_LTO=ON `
      -DCV_TRACE=OFF `
      -DWITH_EIGEN=OFF `
      -DWITH_OPENCL=ON `
      -DWITH_DIRECTX=ON `
      -DWITH_OPENCL_D3D11_NV=ON `
      -DBUILD_EXAMPLES=OFF `
      -DBUILD_TESTS=OFF `
      -DBUILD_opencv_apps=OFF `
      -DBUILD_opencv_ts=OFF `
      -DBUILD_opencv_stitching=OFF `
      -DBUILD_opencv_objdetect=OFF `
      -DBUILD_opencv_python3=OFF `
      -DBUILD_opencv_photo=ON `
      -DBUILD_opencv_gapi=OFF `
      -DBUILD_opencv_dnn=OFF `
      -DBUILD_opencv_ml=OFF `
      ..

# Compile and install OpenCV
cmake --build . --target ALL_BUILD --config "$config"
cmake --install . --prefix ./install/ --config "$config"

# Clone Eigen
$eigen_version = "3.4"
$eigen_path = Join-Path $deps_path "/eigen"
if(-Not (Test-Path $eigen_path))
{
    Set-Location $deps_path
    git clone -b $eigen_version https://gitlab.com/libeigen/eigen.git
}

if(-Not (Test-Path $eigen_path))
{
    Write-Error "Error! Could not clone Eigen."
    exit
}

# Create build folder
$eigen_build_path = Join-Path $eigen_path "/build"
if(-Not (Test-Path $eigen_build_path))
{
    New-Item -Path $eigen_build_path -ItemType Directory
}
Set-Location $eigen_build_path

# Build Eigen (this just generates the cmake package)
cmake ..
cmake --build .


# Clone obs-studio
$obs_studio_version = "27.2.4"
$obs_studio_path = Join-Path $deps_path "/obs-studio"
if(-Not (Test-Path $obs_studio_path))
{
    Set-Location $deps_path
    git clone -b $obs_studio_version --recursive https://github.com/obsproject/obs-studio.git 
}

if(-Not (Test-Path $obs_studio_path))
{
    Write-Error "Error! Could not clone OBS-Studio."
    exit
}


# Download 2019 dependencies (hopefully still hosted by obs-studio)
$obs_studio_deps_path = Join-Path $deps_path "/obs-studio-deps2019/win64"
if(-Not (Test-Path $obs_studio_deps_path))
{
    Set-Location $deps_path
    Invoke-WebRequest -Uri "https://obsproject.com/downloads/dependencies2019.zip" -Outfile "./obs-studio-deps2019.zip"
    Expand-Archive -Path "./obs-studio-deps2019.zip" -DestinationPath "./obs-studio-deps2019"
    Remove-Item -Path "./obs-studio-deps2019.zip"
}
$obs_studio_deps_path = Convert-Path($obs_studio_deps_path)


# Create build folder
$obs_studio_build_path = Join-Path $obs_studio_path "/build"
if(-Not (Test-Path $obs_studio_build_path))
{
    New-Item -Path $obs_studio_build_path -ItemType Directory
}
Set-Location $obs_studio_build_path


# Run CMake configuration
cmake -DDepsPath="$obs_studio_deps_path" `
      -DQTDIR="" `
      -DDISABLE_UI=ON `
      -DENABLE_UI=OFF `
      -DBUILD_BROWSER=OFF `
      -DBUILD_VST=OFF `
      -DENABLE_SCRIPTING=OFF `
      ..

# Compile and install OBS-Studio
cmake --build . --target ALL_BUILD --config "$config"
cmake --install . --prefix ./install/ --config "$config"

# Go back to the original directory
Set-Location $scripts_path
