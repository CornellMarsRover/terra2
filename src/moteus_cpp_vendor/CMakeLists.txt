cmake_minimum_required(VERSION 3.16)
project(moteus_cpp_vendor)
find_package(ament_cmake REQUIRED)
# Header-only mjbots moteus C++ client, vendored from mjbots/moteus at commit
# c37aeed0e617c2bbb261b592f16ef0d44f491e14 (matches the rover's moteus firmware).
add_library(moteus_cpp INTERFACE)
target_include_directories(moteus_cpp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moteus_cpp INTERFACE cxx_std_17)
install(DIRECTORY include/ DESTINATION include)
install(TARGETS moteus_cpp EXPORT export_moteus_cpp)
ament_export_targets(export_moteus_cpp HAS_LIBRARY_TARGET)
ament_package()
