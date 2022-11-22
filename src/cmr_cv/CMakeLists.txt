cmake_minimum_required(VERSION 3.8)
project(cmr_cv)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}"
  "${PROJECT_SOURCE_DIR}/../../cmake")
include(CmrUtils)

# find dependencies
cmr_std_find_pkgs(
  cmr_fabric
  rclcpp_lifecycle
  image_transport
  camera_info_manager
  sensor_msgs
  camera_calibration_parsers
  OpenCV)

# Add additional dependencies as arguments here. Make sure you update
# package.xml if you're depending on other packages.

# Pass "LAUNCH" as an argument if you need a launch directory
cmr_install()

# Add tests with: cmr_add_test(<test_name> SOURCES <test sources> DEPENDS <test
# dependencies>)
cmr_add_node(
  camera_node
  SOURCES
  src/camera_node.cpp
  src/camera_node_main.cpp
  src/external/camera_info_manager.cpp
  DEPENDS
  rclcpp
  std_msgs
  cmr_msgs
  cmr_utils
  cmr_fabric
  rclcpp_lifecycle
  sensor_msgs
  camera_info_manager
  image_transport
  camera_calibration_parsers
  OpenCV)

# PHOBOS_APPEND
cmr_export()

ament_package()
