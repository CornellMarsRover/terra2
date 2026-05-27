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
  msg/ControllerReading.msg
  msg/AutonomyDrive.msg
  msg/ArmJointEffort.msg
  msg/Error.msg
  msg/MotorWriteBatch.msg
  msg/MiniArmDegree.msg
  msg/MotorData.msg
  msg/MotorReadData.msg
  msg/Power.msg
  msg/SensorReadBatch.msg
  msg/IMUSensorData.msg
  msg/Int16ArrayStamped.msg
  msg/GroundPlaneStamped.msg
  # Astrotech (URC 2026) — see gui/README.md
  msg/RamanSpectrum.msg
  msg/EnvSample.msg
  msg/AugerState.msg
  msg/AugerCommand.msg
  msg/AnalysisStatus.msg
  srv/RecoverFault.srv
  srv/ActivateNode.srv
  srv/DeactivateNode.srv
  srv/AcquireDependency.srv
  srv/ReleaseDependency.srv
  srv/NotifyDeactivate.srv
  srv/SiteAnalyze.srv
  srv/SetMixingServoAngle.srv
  srv/StartAnalysisSequence.srv
  action/TestTargetPosition.action
  action/RunAnalysisSequence.action
  DEPENDENCIES
  std_msgs
  geometry_msgs)

ament_package()
