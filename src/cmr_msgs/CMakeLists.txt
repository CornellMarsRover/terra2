cmake_minimum_required(VERSION 3.8)
project(cmr_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add paths to additional messages, services, and actions here.
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  msg/State.msg
  msg/ArucoMarkers.msg
  msg/JoystickReading.msg
  msg/ArmJointEffort.msg
  msg/Error.msg
  msg/MotorWriteBatch.msg
  msg/Power.msg
  msg/SensorReadBatch.msg
  msg/Int16ArrayStamped.msg
  srv/RecoverFault.srv
  srv/ActivateNode.srv
  srv/DeactivateNode.srv
  srv/AcquireDependency.srv
  srv/ReleaseDependency.srv
  srv/NotifyDeactivate.srv
  srv/SiteAnalyze.srv
  action/TestTargetPosition.action
  DEPENDENCIES
  std_msgs
  geometry_msgs)

ament_package()
