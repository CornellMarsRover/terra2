find_package(ament_cmake REQUIRED)

# Creates a new test. This function is a NOP if `BUILD_TESTING` is not enabled
#
# ## Args:
#
# * <name> - the name of the test. The full test name will be
#   <PROJECT_NAME>_<name>
# * <SOURCES> - source files for the test
# * <DEPENDS> - dependencies of the test
#
# ## Example:
#
# `cmr_test(error_test SOURCES test/error_test.cpp test/error_test_helper.cpp
# DEPENDS rclcpp)`
function(cmr_add_test name)
  set(bool_args "")
  set(one_val_args "")
  set(multi_val_args "SOURCES" "DEPENDS")
  cmake_parse_arguments(MK_TEST "${bool_args}" "${one_val_args}"
                        "${multi_val_args}" ${ARGN})

  if(BUILD_TESTING)
    ament_add_gmock(${PROJECT_NAME}_${name} ${MK_TEST_SOURCES})
    ament_target_dependencies(${PROJECT_NAME}_${name} ${MK_TEST_DEPENDS})
    target_include_directories(
      ${PROJECT_NAME}_${name}
      PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
             $<INSTALL_INTERFACE:include>)
    target_compile_options(
      ${PROJECT_NAME}_${name}
      PUBLIC --coverage ${CMR_COMPILE_FLAGS} -Wno-deprecated-copy
             -Wno-gnu-zero-variadic-macro-arguments)
    target_link_options(${PROJECT_NAME}_${name} PUBLIC --coverage)
  endif()
endfunction()

# Adds a library. Sets default include directories and compile/link options.
# This function also sets the build location to install directory.
#
# ## Args:
#
# * <name> - name of the build target
# * <SHARED> - optional flag, if set, builds a shared library by passing SHARED
#   to add_library
# * <SOURCES> - multivalue argument of all source files for the node
# * <DEPENDS> - multivalue argument of all dependencies for the node
#
# ## Examples:
#
# * `cmr_add_lib(cmr_utils SOURCES src/cmr_debug.cpp DEPENDS rclcpp)`
# * `cmr_add_lib(cmr_utils SHARED SOURCES src/cmr_debug.cpp DEPENDS rclcpp)`
function(cmr_add_lib name)
  set(bool_args "SHARED")
  set(one_val_args "")
  set(multi_val_args "SOURCES" "DEPENDS")
  cmake_parse_arguments(MK_LIB "${bool_args}" "${one_val_args}"
                        "${multi_val_args}" ${ARGN})

  if(${MK_LIB_SHARED})
    add_library(${name} SHARED ${MK_LIB_SOURCES})
  else()
    add_library(${name} ${MK_LIB_SOURCES})
  endif()

  ament_target_dependencies(${name} ${MK_LIB_DEPENDS})
  target_include_directories(
    ${name} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                   $<INSTALL_INTERFACE:include>)

  # Require C99 and C++17
  target_compile_features(${name} PUBLIC c_std_99 cxx_std_17)
  target_compile_options(${name} PUBLIC ${CMR_COMPILE_FLAGS})

  if(BUILD_TESTING)
    target_compile_options(${name} PUBLIC --coverage)
    target_link_options(${name} PUBLIC --coverage)
  endif()

  set(CMR_LIBS
      "${CMR_LIBS}" "${name}"
      PARENT_SCOPE)

  install(
    TARGETS ${name}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
endfunction()

# Adds a node or other executable. Sets default include directories and
# compile/link options. This function also sets the build location to the
# install directory.
#
# ## Args:
#
# * <name> - name of the build target
# * <SOURCES> - multivalue argument of all source files for the node
# * <DEPENDS> - multivalue argument of all dependencies for the node
#
# ## Examples:
#
# * `cmr_add_node(demo_node SOURCES src/demo_node.cpp DEPENDS rclcpp cmr_utils)`
function(cmr_add_node name)
  set(bool_args "")
  set(one_val_args "")
  set(multi_val_args "SOURCES" "DEPENDS")
  cmake_parse_arguments(MK_NODE "${bool_args}" "${one_val_args}"
                        "${multi_val_args}" ${ARGN})

  add_executable(${name} ${MK_NODE_SOURCES})
  ament_target_dependencies(${name} ${MK_NODE_DEPENDS})
  target_include_directories(
    ${name} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                   $<INSTALL_INTERFACE:include>)

  # Require C99 and C++17
  target_compile_features(${name} PUBLIC c_std_99 cxx_std_17)
  target_compile_options(${name} PUBLIC ${CMR_COMPILE_FLAGS})

  install(TARGETS ${name} DESTINATION lib/${PROJECT_NAME})

  if(BUILD_TESTING)
    target_compile_options(${name} PUBLIC --coverage)
    target_link_options(${name} PUBLIC --coverage)
  endif()
endfunction()

# Exports the include directory and all libraries added with cmr_add_lib This
# will export a target with the same name as the project if a library is defined
#
# This exports old CMake variables
macro(CMR_EXPORT)
  ament_export_include_directories(include)

  if(NOT "${CMR_LIBS}" STREQUAL "")
    ament_export_targets(export_${PROJECT_NAME})
    ament_export_libraries(${CMR_LIBS})
  endif()
endmacro()

# Installs the include directory for the module and any additional directories
# that are in the project workspace directory (Ex. under `src/cmr_arm` for arm)
#
# ## Args:
#
# * <dir...> - space separated list of directories to install. Can be empty
#
# ## Examples:
#
# * `cmr_install()`
# * `cmr_install(launch)`
# * `cmr_install(launch config)`
function(cmr_install)
  install(DIRECTORY include/ DESTINATION include)

  foreach(dir ${ARGN})
    install(DIRECTORY "${dir}/" DESTINATION "share/${PROJECT_NAME}/${dir}")
  endforeach()
endfunction()

# Calls `find_package` for each argument, passing the REQUIRED option
macro(CMR_FIND_PKGS)
  foreach(pkg ${ARGN})
    find_package(${pkg} REQUIRED)
  endforeach()
endmacro()

# Finds the cmake package of the standard dependencies and all arguments
macro(CMR_STD_FIND_PKGS)
  cmr_find_pkgs(rclcpp std_msgs cmr_msgs cmr_utils ${ARGN})
endmacro()

if(BUILD_TESTING)
  add_compile_options(--coverage)
  add_link_options(--coverage)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  add_compile_definitions(BUILD_TESTS)
endif()

# The following is run for every CMakeLists.txt that includes this module
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMR_COMPILE_FLAGS
      "-Wall"
      "-Wextra"
      "-Wpedantic"
      "-Werror"
      "-Wshadow"
      "-Wconversion"
      "-Wnarrowing"
      "-Wno-unknown-pragmas")
else()
  message(FATAL_ERROR "Unsupported compiler: ${CMAKE_CXX_COMPILER_ID}")
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMR_COMPILE_FLAGS
      ${CMR_COMPILE_FLAGS} "-Wnon-gcc" "-Wsometimes-uninitialized"
      "-Wshadow-all" "-Wunused" "-Wno-unused-function")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMR_LIBS "") # Libraries defined in the file that includes this one
