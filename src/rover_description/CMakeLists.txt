cmake_minimum_required(VERSION 3.8)
project(rover_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(urdf REQUIRED)

install(DIRECTORY config launch rviz urdf DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(xacro)
ament_export_dependencies(urdf)
ament_package()
