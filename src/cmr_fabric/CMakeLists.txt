cmake_minimum_required(VERSION 3.8)
project(cmr_fabric)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED YES)

function (cmr_test NAME)
	set (BOOLEAN_ARGS "")
	set (ONEVALUE_ARGS "")
	set (MULTIVALUE_ARGS "SOURCES" "DEPS")
	cmake_parse_arguments(
		MK_TEST
		"${BOOLEAN_ARGS}"
		"${ONEVALUE_ARGS}"
		"${MULTIVALUE_ARGS}"
		${ARGN}
	)

  ament_add_gtest(${PROJECT_NAME}_${NAME} ${MK_TEST_SOURCES})
  ament_target_dependencies(${PROJECT_NAME}_${NAME} ${MK_TEST_DEPS})
  target_include_directories(${PROJECT_NAME}_${NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_options(${PROJECT_NAME}_${NAME} PUBLIC --coverage)
  target_link_options(${PROJECT_NAME}_${NAME} PUBLIC --coverage)
endfunction()

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
  find_package(ament_cmake_gtest REQUIRED)
  cmr_test(fabric_test SOURCES test/fabric_test.cpp src/fabric_node.cpp 
    DEPS rclcpp rclcpp_lifecycle cmr_msgs cmr_utils)
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
if(BUILD_TESTING)
  target_compile_options(${PROJECT_NAME} PUBLIC --coverage)
  target_link_options(${PROJECT_NAME} PUBLIC --coverage)
endif()

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
if(BUILD_TESTING)
  target_compile_options(fault_handler PUBLIC --coverage)
  target_link_options(fault_handler PUBLIC --coverage)
endif()

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
if(BUILD_TESTING)
  target_compile_options(lifecycle_manager PUBLIC --coverage)
  target_link_options(lifecycle_manager PUBLIC --coverage)
endif()

install(TARGETS lifecycle_manager
  DESTINATION lib/${PROJECT_NAME})
# END NODE lifecycle_manager

# BEGIN NODE dependency_manager
add_executable(dependency_manager
  src/dependency_manager.cpp
)
ament_target_dependencies(dependency_manager
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  std_msgs
  cmr_msgs
  cmr_utils
)
target_include_directories(dependency_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dependency_manager PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
if(BUILD_TESTING)
  target_compile_options(dependency_manager PUBLIC --coverage)
  target_link_options(dependency_manager PUBLIC --coverage)
endif()

install(TARGETS dependency_manager
  DESTINATION lib/${PROJECT_NAME})
# END NODE dependency_manager

# PHOBOS_APPEND

ament_export_include_directories(
  include
)

ament_package()
