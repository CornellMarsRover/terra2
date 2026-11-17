cmake_minimum_required(VERSION 3.8)
project(cmr_zed_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(CMR_ZED_STUB_CHECK
  "Compile (no link, no install) the ZED nodes against test/zed_stub instead of the real ZED SDK"
  OFF)

find_package(ament_cmake REQUIRED)

set(ZED_NODE_SOURCES
  src/zed_camera_publisher.cpp
  src/zed_autonomy.cpp
  src/zed_threaded.cpp
)
set(ZED_MAIN_SOURCES
  src/zed_publisher_node_main.cpp
  src/zed_autonomy_main.cpp
  src/threaded_main.cpp
)
set(ZED_NODE_DEPS
  rclcpp rclcpp_components sensor_msgs std_msgs geometry_msgs cmr_msgs tf2_ros)

if(NOT CMR_ZED_STUB_CHECK)
  find_package(ZED 4 QUIET)
  if(ZED_FOUND)
    find_package(CUDA ${ZED_CUDA_VERSION} QUIET)
  endif()
endif()

if(CMR_ZED_STUB_CHECK OR (ZED_FOUND AND CUDA_FOUND))
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(cmr_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(OpenCV REQUIRED COMPONENTS core)
endif()

if(CMR_ZED_STUB_CHECK)
  # -------------------------------------------------------------------------
  # Compile-only check against the stub SDK header (object files, never
  # linked or installed).
  # -------------------------------------------------------------------------
  message(STATUS "cmr_zed_cpp: CMR_ZED_STUB_CHECK=ON - compiling ZED nodes against test/zed_stub (not linked/installed)")
  add_library(cmr_zed_stub_check OBJECT ${ZED_NODE_SOURCES} ${ZED_MAIN_SOURCES})
  target_include_directories(cmr_zed_stub_check PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/test/zed_stub
    ${OpenCV_INCLUDE_DIRS})
  ament_target_dependencies(cmr_zed_stub_check ${ZED_NODE_DEPS})

elseif(ZED_FOUND AND CUDA_FOUND)
  # -------------------------------------------------------------------------
  # Real build (Jetson with the ZED SDK installed).
  # -------------------------------------------------------------------------
  add_library(cmr_zed_nodes SHARED ${ZED_NODE_SOURCES})
  target_include_directories(cmr_zed_nodes PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_include_directories(cmr_zed_nodes SYSTEM PUBLIC ${ZED_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
  target_link_directories(cmr_zed_nodes PUBLIC ${ZED_LIBRARY_DIR} ${CUDA_LIBRARY_DIRS})
  target_link_libraries(cmr_zed_nodes
    ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${OpenCV_LIBS})
  ament_target_dependencies(cmr_zed_nodes ${ZED_NODE_DEPS})

  # Components (loadable into a container) ...
  rclcpp_components_register_nodes(cmr_zed_nodes
    "cmr_zed_cpp::ZEDCameraPublisher"
    "cmr_zed_cpp::ZedAutonomy"
    "cmr_zed_cpp::ZedThreaded")

  # ... plus hand-written standalone mains that catch constructor failures
  # (log FATAL, exit 1) instead of the generated main's std::terminate().
  add_executable(zed_publisher_node src/zed_publisher_node_main.cpp)
  add_executable(zed_autonomy src/zed_autonomy_main.cpp)
  add_executable(threaded src/threaded_main.cpp)
  foreach(exe zed_publisher_node zed_autonomy threaded)
    target_link_libraries(${exe} cmr_zed_nodes)
    ament_target_dependencies(${exe} ${ZED_NODE_DEPS})
    install(TARGETS ${exe} DESTINATION lib/${PROJECT_NAME})
  endforeach()

  install(TARGETS cmr_zed_nodes
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
  install(DIRECTORY include/ DESTINATION include)

else()
  message(STATUS
    "cmr_zed_cpp: ZED SDK 4 (+ CUDA) not found - building nothing. "
    "Install the ZED SDK to build zed_publisher_node / zed_autonomy / threaded, "
    "or configure with -DCMR_ZED_STUB_CHECK=ON for a compile-only check.")
endif()

ament_package()
