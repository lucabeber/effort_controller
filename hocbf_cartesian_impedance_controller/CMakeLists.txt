cmake_minimum_required(VERSION 3.5)
project(hocbf_cartesian_impedance_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(ADDITIONAL_COMPILE_OPTIONS -Wall -Wextra -Wpedantic -Wno-unused-parameter)
add_compile_options(${ADDITIONAL_COMPILE_OPTIONS})
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
add_definitions(-DEIGEN_MPL2_ONLY)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(effort_controller_base REQUIRED)
set(QPOASES_INSTALL_DIR ${CMAKE_BINARY_DIR}/qpoases_install)
include(ExternalProject)
ExternalProject_Add(
  external_qpoases
  URL https://github.com/coin-or/qpOASES/archive/refs/tags/releases/3.2.1.tar.gz
  CMAKE_ARGS
  -DCMAKE_INSTALL_PREFIX=${QPOASES_INSTALL_DIR}
  -DCMAKE_BUILD_TYPE=Release
  -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
  -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
  -DBUILD_SHARED_LIBS=OFF
  -DQPOASES_BUILD_EXAMPLES=OFF
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON
)
add_library(qpoases_helper INTERFACE)
add_dependencies(qpoases_helper external_qpoases)
target_include_directories(qpoases_helper INTERFACE ${QPOASES_INSTALL_DIR}/include)
target_link_libraries(qpoases_helper INTERFACE ${QPOASES_INSTALL_DIR}/lib/libqpOASES.a)
add_library(qpoases::qpoases ALIAS qpoases_helper)


# Convenience variable for dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
        rclcpp
        effort_controller_base
        Eigen3
)

ament_export_dependencies(
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

#--------------------------------------------------------------------------------
# Libraries
#--------------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/hocbf_cartesian_impedance_controller.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME} qpoases::qpoases)

ament_target_dependencies(${PROJECT_NAME}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

#--------------------------------------------------------------------------------
# Install and export
#--------------------------------------------------------------------------------

pluginlib_export_plugin_description_file(controller_interface hocbf_cartesian_impedance_controller_plugin.xml)

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
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)


ament_package()