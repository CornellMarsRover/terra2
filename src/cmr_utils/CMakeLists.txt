cmake_minimum_required(VERSION 3.8)
project(cmr_utils)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/../../cmake)
include(CmrUtils)

# find dependencies
find_package(rclcpp REQUIRED)

cmr_add_lib(${PROJECT_NAME} SHARED 
  SOURCES src/external/toml.c src/external/tomlcpp.cxx src/cmr_debug.cpp
  DEPS rclcpp)

# Export old-style CMake variables
cmr_export()

cmr_add_test(error_test SOURCES test/error_test.cpp src/cmr_debug.cpp DEPS rclcpp)

ament_package()

cmr_install()
