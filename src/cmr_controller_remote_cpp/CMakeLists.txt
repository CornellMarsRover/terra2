cmake_minimum_required(VERSION 3.16)
project(cmr_controller_remote_cpp LANGUAGES CXX)

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
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Port of cmr_controller_remote/connect.py (entry point connect_node).
add_executable(connect_node src/connect_node.cpp)
ament_target_dependencies(connect_node rclcpp cmr_msgs geometry_msgs sensor_msgs)

install(TARGETS connect_node DESTINATION lib/${PROJECT_NAME})

ament_package()
