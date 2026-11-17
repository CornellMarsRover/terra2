cmake_minimum_required(VERSION 3.16)
project(cmr_servo_control_cpp LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cmr_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

# Ports of CMR_CANFD (FdCanInterface/ServoController) and end_effector_servo_mapper.py.
add_library(servo_control_common STATIC
  src/fd_can_interface.cpp
  src/end_effector_servo_mapper.cpp)
target_include_directories(servo_control_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(servo_control_common PUBLIC Threads::Threads)

add_executable(arm_servo_control_node src/arm_servo_control_node.cpp)
target_link_libraries(arm_servo_control_node servo_control_common)
ament_target_dependencies(arm_servo_control_node rclcpp cmr_msgs)

add_executable(servo_hw_node src/servo_hw_node.cpp)
target_link_libraries(servo_hw_node servo_control_common)
ament_target_dependencies(servo_hw_node rclcpp sensor_msgs)

install(TARGETS arm_servo_control_node servo_hw_node DESTINATION lib/${PROJECT_NAME})

ament_package()
