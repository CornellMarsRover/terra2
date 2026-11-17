cmake_minimum_required(VERSION 3.16)
project(cmr_rtkgps_cpp LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

# Minimal pyubx2 replacement (CFG-VALSET/VALDEL encoding, NAV position decoding).
add_library(ubx_lite STATIC src/ubx.cpp)
target_include_directories(ubx_lite PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

# Port of rover.py (entry point gps_rover).
add_executable(gps_rover src/rover.cpp)
target_link_libraries(gps_rover ubx_lite Threads::Threads)
ament_target_dependencies(gps_rover rclcpp sensor_msgs)

install(TARGETS gps_rover DESTINATION lib/${PROJECT_NAME})

ament_package()
