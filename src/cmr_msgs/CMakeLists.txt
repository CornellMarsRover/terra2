cmake_minimum_required(VERSION 3.8)
project(cmr_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add paths to additional messages, services, and actions here.
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  msg/State.msg
  srv/RecoverFault.srv
  srv/ActivateNode.srv
  srv/DeactivateNode.srv
  srv/AcquireDependency.srv
  srv/ReleaseDependency.srv
  srv/NotifyDeactivate.srv)

ament_package()
