cmake_minimum_required(VERSION 3.8)
project(cmr_fabric)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}"
                      "${PROJECT_SOURCE_DIR}/../../cmake")
include(CmrUtils)

# find dependencies
cmr_std_find_pkgs(rclcpp_lifecycle lifecycle_msgs std_msgs)

# Add additional dependencies here. Make sure you update package.xml if you're
# depending on other packages. find_package(<dependency> REQUIRED)
cmr_add_lib(
  cmr_fabric
  SOURCES
  src/fabric_node.cpp
  src/lifecycle_helpers.cpp
  src/dependency_manager.cpp
  DEPENDS
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  std_msgs
  cmr_msgs
  cmr_utils)

cmr_add_test(
  fabric_test
  SOURCES
  test/fabric_test.cpp
  src/fabric_node.cpp
  src/fault_handler.cpp
  src/dependency_manager.cpp
  src/lifecycle_helpers.cpp
  DEPENDS
  rclcpp
  rclcpp_lifecycle
  cmr_msgs
  cmr_utils
  std_msgs
  lifecycle_msgs)

cmr_add_node(
  fault_handler
  SOURCES
  src/fault_handler.cpp
  src/fault_handler_node.cpp
  src/dependency_manager.cpp
  src/lifecycle_helpers.cpp
  DEPENDS
  rclcpp
  std_msgs
  cmr_msgs
  cmr_utils
  lifecycle_msgs
  rclcpp_lifecycle)

cmr_add_node(
  lifecycle_manager
  SOURCES
  src/lifecycle_helpers.cpp
  src/lifecycle_manager.cpp
  DEPENDS
  rclcpp
  std_msgs
  cmr_msgs
  cmr_utils
  lifecycle_msgs
  rclcpp_lifecycle)

# PHOBOS_APPEND
cmr_install()
cmr_export()

ament_package()
