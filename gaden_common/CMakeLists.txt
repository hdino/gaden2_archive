cmake_minimum_required(VERSION 3.5)
project(gaden_common)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common_config.cmake)
include(gaden_common-extras.cmake)

# find dependencies
#find_package(Eigen3 REQUIRED)
find_package(gaden_eigen REQUIRED)
find_package(OpenVDB 7.1 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)

#ament_export_include_directories(${EIGEN3_INCLUDE_DIRS})

add_library(gaden_common
  src/cad_model.cpp
  src/ros_parameters.cpp
  src/ros_type_helper.cpp
)

target_include_directories(gaden_common PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

target_link_libraries(gaden_common
  OpenVDB::openvdb
  yaml-cpp
)

ament_target_dependencies(gaden_common
  gaden_eigen
  OpenVDB
  rclcpp
  yaml-cpp
)

ament_export_dependencies(
  gaden_eigen
  OpenVDB
)

ament_export_interfaces(export_gaden_common HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

install(
  TARGETS gaden_common
  EXPORT export_gaden_common
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package(
  CONFIG_EXTRAS "gaden_common-extras.cmake"
)
