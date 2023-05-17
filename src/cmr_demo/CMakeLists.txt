cmake_minimum_required(VERSION 3.8)
project(cmr_demo)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}"
  "${PROJECT_SOURCE_DIR}/../../cmake")
include(CmrUtils)

# find dependencies
cmr_std_find_pkgs(cmr_fabric rclcpp_lifecycle sensor_msgs)
pkg_check_modules(libgps REQUIRED libgps)

# Add additional dependencies here. Make sure you update package.xml if you're
# depending on other packages.
cmr_install(launch config)

cmr_add_node(
  demo_node
  SOURCES
  src/demo_node.cpp
  src/demo_node_main.cpp
  DEPENDS
  rclcpp
  rclcpp_lifecycle
  std_msgs
  cmr_msgs
  cmr_fabric
  cmr_utils)

cmr_add_node(gps_fake
  SOURCES
  src/gps_fake.cpp
  src/gps_fake_main.cpp
  DEPENDS
  rclcpp
  std_msgs
  cmr_msgs
  cmr_utils
  cmr_fabric
  rclcpp_lifecycle
  sensor_msgs)

# PHOBOS_APPEND
cmr_export()

ament_package()
