cmake_minimum_required(VERSION 3.8)
project(cmr_fabric)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cmr_msgs REQUIRED)
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

# BEGIN LIBRARY cmr_fabric
add_library(cmr_fabric
  src/fabric_node.cpp
)
ament_target_dependencies(cmr_fabric
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  std_msgs
  cmr_msgs
  cmr_utils
)
target_compile_features(cmr_fabric PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(cmr_fabric PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(
  TARGETS cmr_fabric
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# END LIBRARY cmr_fabric

ament_export_libraries(
  cmr_fabric
)

ament_export_targets(
  export_${PROJECT_NAME}
)

# BEGIN NODE fault_handler
add_executable(fault_handler
  src/fault_handler.cpp
)
ament_target_dependencies(fault_handler
  rclcpp
  std_msgs
  cmr_msgs
  cmr_utils
)
target_include_directories(fault_handler PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(fault_handler PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS fault_handler
  DESTINATION lib/${PROJECT_NAME})
# END NODE fault_handler

# BEGIN NODE lifecycle_manager
add_executable(lifecycle_manager
  src/lifecycle_manager.cpp
)
ament_target_dependencies(lifecycle_manager
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  std_msgs
  cmr_msgs
  cmr_utils
)
target_include_directories(lifecycle_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lifecycle_manager PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS lifecycle_manager
  DESTINATION lib/${PROJECT_NAME})
# END NODE lifecycle_manager

# PHOBOS_APPEND

ament_export_include_directories(
  include
)

ament_package()
