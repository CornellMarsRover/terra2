cmake_minimum_required(VERSION 3.5)
project(usb_camera_publisher)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install entry points
install(
  PROGRAMS
    usb_camera_publisher/publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Register the package
ament_package()
