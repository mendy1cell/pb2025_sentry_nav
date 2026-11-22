cmake_minimum_required(VERSION 3.10)
project(ros2_livox_simulation)

## Gazebo Classic is unavailable on Jazzy. Keep the canonical Livox CSV and
## URDF as installable reference assets while gz_livox_simulation provides the
## supported rotating-ray System implementation.
find_package(ament_cmake_auto REQUIRED)

ament_auto_package(INSTALL_TO_SHARE scan_mode urdf)
