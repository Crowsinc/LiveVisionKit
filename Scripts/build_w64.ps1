param(
    [switch]$plugin=$false,
    [switch]$editor=$false,
    [switch]$install=$false,
    $config="Release"
)

$scripts_path = Get-Location
$project_path = Convert-Path (Join-Path $scripts_path "/../")

# Call setup script
./setup_w64.ps1 -config "$config"

# Create LVK build folder
$build_path = Join-Path $project_path "/Build" 
if(-Not (Test-Path $build_path))
{
    New-Item -Path $build_path -ItemType Directory
}
Set-Location $build_path

# Resolve dependency build paths
$opencv_build_path = Join-Path $project_path "/Dependencies/opencv/build"
$obs_studio_build_path = Join-Path $project_path "/Dependencies/obs-studio/build"

# Run LVK CMake configuration
cmake -DBUILD_OBS_PLUGIN="$plugin" -DBUILD_VIDEO_EDITOR="$editor" -DOBS_PLUGIN_AUTO_INSTALL="$install" `
      -DOBS_BUILD_PATH="$obs_studio_build_path" -DOPENCV_BUILD_PATH="$opencv_build_path" ..


# Compile LiveVisionKit and install it to the build folder. 
cmake --build . --target ALL_BUILD --config "$config"
cmake --install . --config "$config"

# Go back to the original directory
Set-Location $scripts_path
