cmake_minimum_required(VERSION 3.16)
project(cmr_rovernet_cpp LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # Keep IEEE double arithmetic identical to CPython's: GCC on aarch64 (the
  # Jetson) contracts a*b+c into a fused multiply-add by default, which makes
  # the swerve kinematics differ from the Python originals in the last bit.
  add_compile_options(-ffp-contract=off)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cmr_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moteus_cpp_vendor REQUIRED)
find_package(Threads REQUIRED)

# ROS-independent core (mirrors the Python *_core / pure modules): command
# mux selection, drive modes, swerve kinematics + controller decoding.
add_library(rovernet_cpp_core STATIC
  src/command_mux_core.cpp
  src/drive_modes.cpp
  src/drive_common.cpp)
set_target_properties(rovernet_cpp_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(rovernet_cpp_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Shared helpers: rovernet_utils port, tiny TOML reader, POSIX serial, moteus bus.
add_library(rovernet_cpp_common STATIC
  src/rovernet_utils.cpp
  src/toml_lite.cpp
  src/node_config.cpp
  src/serial_port.cpp
  src/moteus_bus.cpp)
target_include_directories(rovernet_cpp_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(rovernet_cpp_common PUBLIC
  rovernet_cpp_core moteus_cpp_vendor::moteus_cpp Threads::Threads)
ament_target_dependencies(rovernet_cpp_common PUBLIC rclcpp)

set(NODES
  cmr_read_node
  armnet_node
  usama_control_testing_node
  drive_only_control_node
  steer_only_control_node
  single_drive_wheel_control_node)

foreach(node ${NODES})
  add_executable(${node} src/${node}.cpp)
  target_link_libraries(${node} rovernet_cpp_common)
  ament_target_dependencies(${node}
    rclcpp cmr_msgs geometry_msgs sensor_msgs std_msgs trajectory_msgs)
endforeach()

# drive_command_mux needs no hardware helpers, only the core.
add_executable(drive_command_mux src/drive_command_mux.cpp)
target_link_libraries(drive_command_mux rovernet_cpp_core)
ament_target_dependencies(drive_command_mux rclcpp cmr_msgs std_msgs)
list(APPEND NODES drive_command_mux)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_command_mux_core test/test_command_mux_core.cpp)
  target_link_libraries(test_command_mux_core rovernet_cpp_core)
  ament_add_gtest(test_drive_modes test/test_drive_modes.cpp)
  target_link_libraries(test_drive_modes rovernet_cpp_core)
  ament_add_gtest(test_swerve_kinematics test/test_swerve_kinematics.cpp)
  target_link_libraries(test_swerve_kinematics rovernet_cpp_core)

  # Prints calculate_swerve_targets / shape_motion_for_mode / clamp results for
  # stdin cases; test/compare_swerve_with_python.py diffs it against the
  # Python originals (not installed, not run by colcon test).
  add_executable(swerve_targets_cli test/swerve_targets_cli.cpp)
  target_link_libraries(swerve_targets_cli rovernet_cpp_core)
endif()

install(TARGETS ${NODES} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
