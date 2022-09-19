cmake_minimum_required(VERSION 3.8)
project(cmr_test)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}"
                      "${PROJECT_SOURCE_DIR}/../../cmake")
include(CmrUtils)

# find dependencies
cmr_std_find_pkgs(tf2 tf2_ros geometry_msgs nav_msgs rclcpp_action)

# Add additional dependencies as arguments here. Make sure you update
# package.xml if you're depending on other packages.

# Pass "LAUNCH" as an argument if you need a launch directory
cmr_install(LAUNCH)

# Add tests with: cmr_add_test(<test_name> SOURCES <test sources> DEPENDS <test
# dependencies>)
cmr_add_node(
  test_tf2
  SOURCES
  src/test_tf2.cpp
  DEPENDS
  rclcpp
  std_msgs
  cmr_msgs
  cmr_utils
  tf2
  tf2_ros
  geometry_msgs
  nav_msgs
  rclcpp_action)

cmr_add_node(
  test_controller
  SOURCES
  src/test_controller.cpp
  DEPENDS
  rclcpp
  std_msgs
  cmr_msgs
  cmr_utils
  geometry_msgs
  rclcpp_action)

# PHOBOS_APPEND
cmr_export()

ament_package()
