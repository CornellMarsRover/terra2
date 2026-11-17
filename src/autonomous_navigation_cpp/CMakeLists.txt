cmake_minimum_required(VERSION 3.8)
project(autonomous_navigation_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # -ffp-contract=off: no FMA fusion, so float math rounds like numpy/CPython
  # (py_hypot in py_compat.hpp depends on it).
  add_compile_options(-Wall -Wextra -Wpedantic -ffp-contract=off -Wno-psabi)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(class_loader REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cmr_msgs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

# ---------------------------------------------------------------------------
# ROS-free algorithm library (unit tested); mirrors the Python *_core modules.
# ---------------------------------------------------------------------------
add_library(nav_core STATIC
  src/costmap_core.cpp
  src/planner_core.cpp
  src/local_planner_core.cpp
  src/state_machine_core.cpp
)
set_target_properties(nav_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(nav_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(nav_core PUBLIC Eigen3::Eigen)

# ---------------------------------------------------------------------------
# Composable nodes (one shared library each) + standalone executables with the
# same names as the Python entry points.
# ---------------------------------------------------------------------------
set(node_deps rclcpp rclcpp_components std_msgs geometry_msgs sensor_msgs
  builtin_interfaces cmr_msgs)

function(nav_component target src plugin exe)
  add_library(${target} SHARED ${src})
  target_link_libraries(${target} nav_core ${ARGN})
  ament_target_dependencies(${target} ${node_deps})
  rclcpp_components_register_nodes(${target} ${plugin})

  set(NAV_EXECUTABLE ${exe})
  set(NAV_PLUGIN ${plugin})
  set(NAV_LIBRARY ${CMAKE_SHARED_LIBRARY_PREFIX}${target}${CMAKE_SHARED_LIBRARY_SUFFIX})
  configure_file(src/node_main.cpp.in
    ${CMAKE_CURRENT_BINARY_DIR}/node_main_${exe}.cpp @ONLY)
  add_executable(${exe} ${CMAKE_CURRENT_BINARY_DIR}/node_main_${exe}.cpp)
  ament_target_dependencies(${exe} rclcpp rclcpp_components class_loader)

  install(TARGETS ${target}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
  install(TARGETS ${exe} DESTINATION lib/${PROJECT_NAME})
endfunction()

nav_component(costmap_component src/costmap_node.cpp
  "autonomous_navigation_cpp::CostmapNode" costmap)
nav_component(local_planner_component src/local_planner_node.cpp
  "autonomous_navigation_cpp::LocalPlannerNode" local_planner)
nav_component(controller_component src/controller_node.cpp
  "autonomous_navigation_cpp::ControllerNode" controller)
nav_component(kalman_localization_component src/kalman_localization_node.cpp
  "autonomous_navigation_cpp::KalmanLocalizationNode" kalman_localization)
nav_component(new_kalman_component src/new_kalman_node.cpp
  "autonomous_navigation_cpp::NewKalmanNode" new_kalman)
nav_component(state_machine_component src/state_machine_node.cpp
  "autonomous_navigation_cpp::StateMachineNode" state_machine
  yaml-cpp ament_index_cpp::ament_index_cpp)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_nav_core
    test/test_costmap_core.cpp
    test/test_drive_command.cpp
    test/test_planner_core.cpp
    test/test_state_machine_core.cpp
    test/test_target_contract.cpp
    test/test_kalman.cpp)
  target_link_libraries(test_nav_core nav_core)
endif()

ament_package()
