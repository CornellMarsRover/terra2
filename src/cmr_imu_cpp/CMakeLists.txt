cmake_minimum_required(VERSION 3.16)
project(cmr_imu_cpp LANGUAGES CXX)

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
find_package(Threads REQUIRED)

# Port of device_model.py / protocol_485_resolver.py / jy901s_dataProcessor.py.
add_library(jy901s_protocol STATIC
  src/device_model.cpp
  src/protocol_485_resolver.cpp)
target_include_directories(jy901s_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(jy901s_protocol PUBLIC Threads::Threads)

# Port of imu.py (entry point imu_node).
add_executable(imu_node src/imu_node.cpp)
target_link_libraries(imu_node jy901s_protocol)
ament_target_dependencies(imu_node rclcpp cmr_msgs)

install(TARGETS imu_node DESTINATION lib/${PROJECT_NAME})

ament_package()
