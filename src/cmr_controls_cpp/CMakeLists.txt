cmake_minimum_required(VERSION 3.16)
project(cmr_controls_cpp LANGUAGES CXX)

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
find_package(std_msgs REQUIRED)
find_package(moteus_cpp_vendor REQUIRED)
find_package(Threads REQUIRED)

# Port of swerve_controller_node.py
add_executable(swerve_controller_node src/swerve_controller_node.cpp)
target_include_directories(swerve_controller_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(swerve_controller_node moteus_cpp_vendor::moteus_cpp Threads::Threads)
ament_target_dependencies(swerve_controller_node rclcpp cmr_msgs)

# Port of arm_controller_node.py
add_executable(arm_controller_node src/arm_controller_node.cpp)
target_include_directories(arm_controller_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(arm_controller_node moteus_cpp_vendor::moteus_cpp Threads::Threads)
ament_target_dependencies(arm_controller_node rclcpp std_msgs)

# ik_node.py is intentionally not ported (not launched anywhere; see package report).

install(TARGETS swerve_controller_node arm_controller_node DESTINATION lib/${PROJECT_NAME})

ament_package()
