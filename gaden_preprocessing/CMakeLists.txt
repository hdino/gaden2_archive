cmake_minimum_required(VERSION 3.5)
project(gaden_preprocessing)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common_config.cmake)

# find dependencies
find_package(gaden_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(preprocessing
  src/main.cpp
  src/occupancy_grid.cpp
  src/preprocessing.cpp
  src/stl_data.cpp
  src/stl_format.cpp
  src/stl_to_grid.cpp
)

target_include_directories(preprocessing PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

target_link_libraries(preprocessing
  gaden_common::gaden_common
  yaml-cpp
)

ament_target_dependencies(preprocessing
  gaden_common
  rclcpp
  yaml-cpp
)

install(TARGETS preprocessing
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
