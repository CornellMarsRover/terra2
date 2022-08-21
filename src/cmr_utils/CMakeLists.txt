cmake_minimum_required(VERSION 3.8)
project(cmr_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED YES)

# Creates a new test
# Args: test NAME
#       test SOURCES
#       test DEPENDENCIES
# 
# Example:
# cmr_test(error_test SOURCES test/error_test.cpp test/error_test_helper.cpp DEPS rclcpp)
function (cmr_test NAME)
	set (BOOLEAN_ARGS "")
	set (ONEVALUE_ARGS "")
	set (MULTIVALUE_ARGS "SOURCES" "DEPS")
	cmake_parse_arguments(
		MK_TEST
		"${BOOLEAN_ARGS}"
		"${ONEVALUE_ARGS}"
		"${MULTIVALUE_ARGS}"
		${ARGN}
	)

  ament_add_gtest(${PROJECT_NAME}_${NAME} ${MK_TEST_SOURCES})
  ament_target_dependencies(${PROJECT_NAME}_${NAME} ${MK_TEST_DEPS})
  target_include_directories(${PROJECT_NAME}_${NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
endfunction()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/toml.c
  src/tomlcpp.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Export old-style CMake variables
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  #find_package(ament_lint_auto REQUIRED)
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()

  cmr_test(error_test SOURCES test/error_test.cpp DEPS rclcpp)
endif()

ament_package()

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
)