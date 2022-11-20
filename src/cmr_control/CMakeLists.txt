cmake_minimum_required(VERSION 3.8)
project(cmr_control)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}"
                      "${PROJECT_SOURCE_DIR}/../../cmake")
include(CmrUtils)

# Add additional dependencies as arguments here. Make sure you update
# package.xml if you're depending on other packages. find dependencies
cmr_std_find_pkgs(rclcpp_lifecycle hardware_interface pluginlib)

# Add other directories to install here
cmr_install(launch config)

# Add tests with: cmr_add_test(<test_name> SOURCES <test sources> DEPENDS <test
# dependencies>)
cmr_add_lib(
  ${PROJECT_NAME}
  SHARED
  SOURCES
  src/drives_system.cpp
  src/base_arm_system.cpp
  src/arm_system.cpp
  src/mock_arm_system.cpp
  DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  cmr_utils)

# Export hardware pligins
pluginlib_export_plugin_description_file(hardware_interface hardware.xml)

# Export old-style CMake variables
cmr_export()

ament_package()
