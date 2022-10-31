cmake_minimum_required(VERSION 3.8)
project(cmr_arm)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${PROJECT_SOURCE_DIR}/../../cmake")
include(CmrUtils)

# find dependencies
cmr_std_find_pkgs(cmr_fabric rclcpp_lifecycle)

# Add additional dependencies as arguments here. Make sure you update package.xml if you're depending
# on other packages.

# Pass "LAUNCH" as an argument if you need a launch directory
cmr_install()

# Add tests with:
# cmr_add_test(<test_name> SOURCES <test sources> DEPENDS <test dependencies>)
cmr_add_node(joystick_direct_control
  SOURCES
  src/joystick_direct_control.cpp
  src/joystick_direct_control_main.cpp
  DEPENDS
  rclcpp
  std_msgs
  cmr_msgs
  cmr_utils
  cmr_fabric
  rclcpp_lifecycle)

# PHOBOS_APPEND
cmr_export()

ament_package()
