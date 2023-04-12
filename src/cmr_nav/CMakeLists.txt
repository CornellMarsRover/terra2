cmake_minimum_required(VERSION 3.8)
project(cmr_nav)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${PROJECT_SOURCE_DIR}/../../cmake")
include(CmrUtils)

remove(CMR_COMPILE_FLAGS -Werror)

# find dependencies
cmr_std_find_pkgs(cmr_fabric rclcpp_lifecycle geometry_msgs rclcpp_action sensor_msgs
  nav2_common nav2_core nav2_util nav2_msgs nav_msgs nav2_waypoint_follower nav2_lifecycle_manager
  nav2_behavior_tree pluginlib behaviortree_cpp_v3)

# Add additional dependencies as arguments here. Make sure you update package.xml if you're depending
# on other packages.

# Pass "LAUNCH" as an argument if you need a launch directory
# cmr_install(LAUNCH)

# Add tests with:
# cmr_add_test(<test_name> SOURCES <test sources> DEPENDS <test dependencies>)
cmr_add_node(autonomy_control_node
  SOURCES
  src/autonomy_control_node.cpp
  src/autonomy_control_node_main.cpp
  DEPENDS
  rclcpp
  std_msgs
  cmr_msgs
  cmr_utils
  cmr_fabric
  rclcpp_lifecycle
  geometry_msgs
  rclcpp_action
  sensor_msgs
  nav2_common
  nav2_core
  nav2_util
  nav2_msgs
  nav2_waypoint_follower
  nav2_lifecycle_manager)

# cmr_add_node(bt_navigate
# SOURCES
# src/bt_navigate.cpp
# src/bt_navigate_main.cpp
# DEPENDS
# rclcpp
# std_msgs
# cmr_msgs
# cmr_utils
# cmr_fabric
# rclcpp_lifecycle)

# add_compile_options(--compile-no-warning-as-error)
cmr_add_lib(ar_tag_navigate SHARED
  SOURCES
  src/ar_tag_navigate.cpp
  DEPENDS
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  geometry_msgs
  nav2_behavior_tree
  nav_msgs
  nav2_msgs
  behaviortree_cpp_v3
  nav2_util
  nav2_core
  pluginlib)

pluginlib_export_plugin_description_file(nav2_core navigator_plugins.xml)

cmr_add_lib(cmr_print_node SHARED
  SOURCES
  src/print_node.cpp
  DEPENDS
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  nav2_msgs
  nav_msgs
  behaviortree_cpp_v3
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  std_srvs
  nav2_util
)
target_compile_definitions(cmr_print_node PRIVATE BT_PLUGIN_EXPORT)

cmr_add_lib(cmr_pose_generator_node SHARED
  SOURCES
  src/pose_generator_node.cpp
  DEPENDS
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  nav2_msgs
  nav_msgs
  behaviortree_cpp_v3
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  std_srvs
  nav2_util
)
target_compile_definitions(cmr_pose_generator_node PRIVATE BT_PLUGIN_EXPORT)

cmr_add_lib(cmr_square_path_node SHARED
  SOURCES
  src/square_path_node.cpp
  DEPENDS
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  nav2_msgs
  nav_msgs
  behaviortree_cpp_v3
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  std_srvs
  nav2_util
)
target_compile_definitions(cmr_square_path_node PRIVATE BT_PLUGIN_EXPORT)

# PHOBOS_APPEND
cmr_export()

ament_package()
