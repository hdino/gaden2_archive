cmake_minimum_required(VERSION 3.5)
project(gaden_environment)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common_config.cmake)

# find dependencies
find_package(gaden_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(environment
  src/environment.cpp
  src/grid_to_marker.cpp
  src/main.cpp
)

target_include_directories(environment PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

target_link_libraries(environment
  gaden_common::gaden_common
  yaml-cpp
)

ament_target_dependencies(environment
  gaden_common
  rclcpp
  tf2
  visualization_msgs
  yaml-cpp
)

install(TARGETS environment
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
