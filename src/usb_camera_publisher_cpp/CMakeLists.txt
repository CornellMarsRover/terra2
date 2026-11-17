cmake_minimum_required(VERSION 3.8)
project(usb_camera_publisher_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio calib3d)
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST QUIET IMPORTED_TARGET gstreamer-1.0 gstreamer-app-1.0)

# ---------------------------------------------------------------------------
# OpenCV-based camera nodes (image_publisher, bev, stitched)
# ---------------------------------------------------------------------------
add_library(usb_camera_nodes SHARED
  src/image_publisher_node.cpp
  src/bev_node.cpp
  src/stitched_node.cpp
)
target_include_directories(usb_camera_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(usb_camera_nodes ${OpenCV_LIBS})
ament_target_dependencies(usb_camera_nodes
  rclcpp rclcpp_components sensor_msgs std_msgs cv_bridge)

rclcpp_components_register_node(usb_camera_nodes
  PLUGIN "usb_camera_publisher_cpp::MultiCameraImagePublisher"
  EXECUTABLE image_publisher)
rclcpp_components_register_node(usb_camera_nodes
  PLUGIN "usb_camera_publisher_cpp::BirdseyePublisher"
  EXECUTABLE bev)
rclcpp_components_register_node(usb_camera_nodes
  PLUGIN "usb_camera_publisher_cpp::MultiCameraStitchedPublisher"
  EXECUTABLE stitched)

install(TARGETS usb_camera_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# ---------------------------------------------------------------------------
# GStreamer x264 publisher (`publisher`). Needs gstreamer-1.0 + gstreamer-app-1.0
# development files; skipped (with a message) when they are missing.
# ---------------------------------------------------------------------------
if(GST_FOUND)
  add_library(x264_publisher_node SHARED src/x264_publisher_node.cpp)
  target_link_libraries(x264_publisher_node PkgConfig::GST yaml-cpp)
  ament_target_dependencies(x264_publisher_node
    rclcpp rclcpp_components sensor_msgs foxglove_msgs)
  rclcpp_components_register_node(x264_publisher_node
    PLUGIN "usb_camera_publisher_cpp::X264MultiCameraPublisher"
    EXECUTABLE publisher)
  install(TARGETS x264_publisher_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
else()
  message(WARNING
    "usb_camera_publisher_cpp: gstreamer-1.0 / gstreamer-app-1.0 dev files not found; "
    "the `publisher` executable will NOT be built "
    "(apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev).")
endif()

install(DIRECTORY include/ DESTINATION include)

ament_package()
