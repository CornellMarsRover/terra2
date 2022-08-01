cmake_minimum_required(VERSION 3.8)
project(cmr_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cmr_msgs REQUIRED)
find_package(cmr_fabric REQUIRED)
find_package(cmr_utils REQUIRED)
# Add additional dependencies here. Make sure you update package.xml if you're depending
# on other packages.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# BEGIN NODE demo_node
add_executable(demo_node
  src/demo_node.cpp
)
ament_target_dependencies(demo_node
  rclcpp
  rclcpp_lifecycle
  std_msgs
  cmr_msgs
  cmr_fabric
  cmr_utils
)
target_include_directories(demo_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(demo_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS demo_node
  DESTINATION lib/${PROJECT_NAME})
# END NODE demo_node

# PHOBOS_APPEND

ament_export_include_directories(
  include
)

ament_package()
