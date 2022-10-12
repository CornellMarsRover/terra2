cmake_minimum_required(VERSION 3.8)
project(cmr_control)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${PROJECT_SOURCE_DIR}/../../cmake")
include(CmrUtils)

# find dependencies
cmr_std_find_pkgs(rclcpp_lifecycle hardware_interface pluginlib)

# Add additional dependencies as arguments here. Make sure you update package.xml if you're depending
# on other packages.
cmr_add_lib(
    cmr_control
    SOURCES
    src/drives_system.cpp
    DEPENDS
    rclcpp
    rclcpp_lifecycle
    std_msgs
    cmr_msgs
    cmr_utils
    hardware_interface
    pluginlib)

# Pass "LAUNCH" as an argument if you need a launch directory
cmr_install()

# Add tests with:
# cmr_add_test(<test_name> SOURCES <test sources> DEPENDS <test dependencies>)

# PHOBOS_APPEND
cmr_export()

ament_package()
