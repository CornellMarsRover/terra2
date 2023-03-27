cmake_minimum_required(VERSION 3.8)
project(cmr_control)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}"
                      "${PROJECT_SOURCE_DIR}/../../cmake")
include(CmrUtils)

# Add additional dependencies as arguments here. Make sure you update
# package.xml if you're depending on other packages. find dependencies
cmr_std_find_pkgs(
  rclcpp_lifecycle hardware_interface pluginlib generate_parameter_library
  controller_interface realtime_tools)

# Add other directories to install here
cmr_install(launch config)

# Converts YAML configuration parameters into source code that defines a struct
# which can be used from the AstroSensorBroadcaster class
generate_parameter_library(astro_sensor_broadcaster_params
                           src/astro_sensor_broadcaster_params.yaml)

# Add tests with: cmr_add_test(<test_name> SOURCES <test sources> DEPENDS <test
# dependencies>)
cmr_add_lib(
  ${PROJECT_NAME}
  SHARED
  NO_INSTALL
  SOURCES
  src/drives_system.cpp
  src/base_arm_system.cpp
  src/arm_system.cpp
  src/mock_arm_system.cpp
  src/astro_sensor_broadcaster.cpp
  src/astro_sensor.cpp
  src/astro_motor.cpp
  DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  cmr_utils
  cmr_msgs
  controller_interface)

target_link_libraries(${PROJECT_NAME} astro_sensor_broadcaster_params)
install(
  TARGETS ${PROJECT_NAME} astro_sensor_broadcaster_params
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Manually add astro_sensor_broadcaster_params to the list of libraries this
# CMake will export
list(APPEND CMR_LIBS astro_sensor_broadcaster_params)

# Export hardware pligins
pluginlib_export_plugin_description_file(hardware_interface hardware.xml)

# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface controllers.xml)

# Export old-style CMake variables
cmr_export()

ament_package()
