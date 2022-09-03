find_package(ament_cmake REQUIRED)

# Creates a new test.
# This function is a NOP if `BUILD_TESTING` is not enabled
# Args: 
#   <NAME> - the name of the test. The full test name will be <PROJECT_NAME>_<NAME>
#   <SOURCES> - source files for the test
#   <DEPS>  - dependencies of the test
# 
# Example:
#   cmr_test(error_test SOURCES test/error_test.cpp test/error_test_helper.cpp DEPS rclcpp)
function (cmr_add_test NAME)
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

    if(BUILD_TESTING)
        ament_add_gtest(${PROJECT_NAME}_${NAME} ${MK_TEST_SOURCES})
        ament_target_dependencies(${PROJECT_NAME}_${NAME} ${MK_TEST_DEPS})
        target_include_directories(${PROJECT_NAME}_${NAME} PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)
        target_compile_options(${PROJECT_NAME}_${NAME} PUBLIC --coverage)
        target_link_options(${PROJECT_NAME}_${NAME} PUBLIC --coverage)
    endif()
endfunction()


# Adds a library
# Sets default include directories and compile/link options
# This function also sets the build location to install directory
# Args:
#   <NAME> - name of the build target
#   <SHARED> - optional flag, if set, builds a shared library by passing 
#       SHARED to add_library
#   <SOURCES> - multivalue argument of all source files for the node
#   <DEPS> - multivalue argument of all dependencies for the node
# Examples:
#   cmr_add_lib(cmr_utils SOURCES src/cmr_debug.cpp DEPS rclcpp)
#   cmr_add_lib(cmr_utils SHARED SOURCES src/cmr_debug.cpp DEPS rclcpp)
function (cmr_add_lib NAME)
	set (BOOLEAN_ARGS "SHARED")
	set (ONEVALUE_ARGS "")
	set (MULTIVALUE_ARGS "SOURCES" "DEPS")
	cmake_parse_arguments(
		MK_LIB
		"${BOOLEAN_ARGS}"
		"${ONEVALUE_ARGS}"
		"${MULTIVALUE_ARGS}"
		${ARGN}
	)

    if(${MK_LIB_SHARED})
        add_library(${NAME} SHARED ${MK_LIB_SOURCES})
    else()
        add_library(${NAME} ${MK_LIB_SOURCES})
    endif()
  
    ament_target_dependencies(${NAME} ${MK_LIB_DEPS})
    target_include_directories(${NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

    target_compile_features(${NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

    if(BUILD_TESTING)
        target_compile_options(${NAME} PUBLIC --coverage)
        target_link_options(${NAME} PUBLIC --coverage)
    endif()

    set(CMR_LIBS "${CMR_LIBS}" "${NAME}" PARENT_SCOPE)

    install(
        TARGETS ${NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )

endfunction()

# Adds a node or other executable 
# Sets default include directories and compile/link options
# This function also sets the build location to install directory
# Args:
#   <NAME> - name of the build target
#   <SOURCES> - multivalue argument of all source files for the node
#   <DEPS> - multivalue argument of all dependencies for the node
# Examples:
#   cmr_add_node(demo_node SOURCES src/demo_node.cpp DEPS rclcpp cmr_utils)
function (cmr_add_node NAME)
	set (BOOLEAN_ARGS "")
	set (ONEVALUE_ARGS "")
	set (MULTIVALUE_ARGS "SOURCES" "DEPS")
	cmake_parse_arguments(
		MK_NODE
		"${BOOLEAN_ARGS}"
		"${ONEVALUE_ARGS}"
		"${MULTIVALUE_ARGS}"
		${ARGN}
	)

    add_executable(${NAME} ${MK_NODE_SOURCES})
    ament_target_dependencies(${NAME} ${MK_NODE_DEPS})
    target_include_directories(${NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
    target_compile_features(${NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

    install(TARGETS ${NAME} DESTINATION lib/${PROJECT_NAME})

    if(BUILD_TESTING)
        target_compile_options(${NAME} PUBLIC --coverage)
        target_link_options(${NAME} PUBLIC --coverage)
    endif()

endfunction()

# Exports the include directory and all libraries added with
# cmr_add_lib
# This will export a target with the same name as the project
# if a library is defined
#
# This exports old CMake variables
macro(cmr_export)
    ament_export_include_directories(include)
    if(NOT "${CMR_LIBS}" STREQUAL "")
        ament_export_targets(export_${PROJECT_NAME})
        ament_export_libraries(${CMR_LIBS})
    endif()
endmacro()

# Installs the include directory for the module
# Args:
#   <LAUNCH> - optional flag, if passed will also install the launch directory
# Examples:
#   cmr_install()
#   cmr_install(LAUNCH)
function(cmr_install)
    cmake_parse_arguments(
		CMR_INSTALL
		"LAUNCH"
		""
		""
		${ARGN}
	)

    install(
        DIRECTORY include/
        DESTINATION include
    )
  
    if(${CMR_INSTALL_LAUNCH})
        install(
            DIRECTORY launch/
            DESTINATION share/${PROJECT_NAME}/launch
        )
    endif()
endfunction()

# Calls `find_package` for each argument
# passes the REQUIRED option
macro(cmr_find_pkgs)
    foreach(PACKAGE ${ARGN})
        find_package(${PACKAGE} REQUIRED)
    endforeach()
endmacro()

# Finds the cmake package of the standard dependencies and all
# arguments
macro(cmr_std_find_pkgs)
    cmr_find_pkgs(rclcpp std_msgs cmr_msgs cmr_utils ${ARGN})
endmacro()

# The following is run for every CMakeLists.txt that includes this module

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
else()
    message(FATAL_ERROR "Unsupported compiler: ${CMAKE_CXX_COMPILER_ID}")
endif()

if(BUILD_TESTING)
    add_compile_options(--coverage)
    add_link_options(--coverage)
    find_package(ament_cmake_gtest REQUIRED)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMR_LIBS "") # Libraries defined in the file that includes this one