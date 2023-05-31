cmake_minimum_required(VERSION 3.8)
project(cmr_cv)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}"
  "${PROJECT_SOURCE_DIR}/../../cmake")
include(CmrUtils)

# Hack for now
remove(CMR_COMPILE_FLAGS -Werror)

# find dependencies
cmr_std_find_pkgs(
  cmr_fabric
  rclcpp_lifecycle
  image_transport
  camera_info_manager
  sensor_msgs
  camera_calibration_parsers
  OpenCV
  geometry_msgs
  nav2_behavior_tree
  nav_msgs
  nav2_msgs
  behaviortree_cpp_v3
  nav2_util
  nav2_core
  pluginlib
  rclcpp_action
  nav2_common
  nav2_waypoint_follower
  nav2_lifecycle_manager
  tf2_geometry_msgs
  tf2
  tf2_ros)

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

# Aruco node behavior tree action plugin
cmr_add_lib(
  aruco_bt_action
  SHARED
  SOURCES
  src/aruco_bt_action.cpp
  DEPENDS
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  sensor_msgs
  std_srvs
  std_msgs
  cmr_msgs
  geometry_msgs
  nav2_behavior_tree
  nav_msgs
  nav2_msgs
  behaviortree_cpp_v3
  nav2_util
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_core
  pluginlib)
target_compile_definitions(aruco_bt_action PRIVATE BT_PLUGIN_EXPORT)

cmr_add_node(pan_tilt_node 
  SOURCES 
  src/pan_tilt_node.cpp
  src/pan_tilt_node_main.cpp
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
