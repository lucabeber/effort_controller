cmake_minimum_required(VERSION 3.5)
project(cartesian_impedance_controller)

set(CMAKE_CXX_STANDARD 17)
set(ADDITIONAL_COMPILE_OPTIONS -Wall -Wextra -Wpedantic -Wno-unused-parameter)

# Optional parameter to enable logging, only if MatLogger2 is found
# Refer to https://github.com/ADVRHumanoids/MatLogger2 to install MatLogger2
# output file is /tmp/cart_impedance_log<timestamp>.mat
option(LOGGING "Enable logging" ON)
add_compile_options(${ADDITIONAL_COMPILE_OPTIONS})

# Build in release mode if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
add_definitions(-DEIGEN_MPL2_ONLY)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(effort_controller_base REQUIRED)
find_package(debug_msg REQUIRED)

# Try to find MatLogger2 if logging is enabled
if (LOGGING)
  find_package(matlogger2 QUIET)
  find_package(lbr_fri_idl)
  if (matlogger2_FOUND)
    message(STATUS "MatLogger2 found: Logging enabled")
    set(LOGGING_ENABLED TRUE)
  else()
    message(WARNING "MatLogger2 not found: Logging disabled")
    set(LOGGING_ENABLED FALSE)
  endif()
else()
  set(LOGGING_ENABLED FALSE)
endif()

# Convenience variable for dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
        rclcpp
        effort_controller_base
        Eigen3
        debug_msg
)

if (LOGGING_ENABLED)
  list(APPEND THIS_PACKAGE_INCLUDE_DEPENDS matlogger2)
  list(APPEND THIS_PACKAGE_INCLUDE_DEPENDS lbr_fri_idl)
  add_compile_definitions(LOGGING)
endif()

ament_export_dependencies(
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

#--------------------------------------------------------------------------------
# Libraries
#--------------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/cartesian_impedance_controller.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

if (LOGGING_ENABLED)
  target_link_libraries(${PROJECT_NAME} matlogger2::matlogger2)
endif()

#--------------------------------------------------------------------------------
# Install and export
#--------------------------------------------------------------------------------

pluginlib_export_plugin_description_file(controller_interface cartesian_impedance_controller_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()