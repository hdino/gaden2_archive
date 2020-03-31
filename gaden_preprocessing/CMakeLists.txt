cmake_minimum_required(VERSION 3.5)
project(gaden_preprocessing)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common_config.cmake)

# find dependencies
find_package(gaden_common REQUIRED)
find_package(PNG REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# preprocessing
add_executable(preprocessing
  src/main.cpp
  src/png_exporter.cpp
  src/preprocessing.cpp
  src/stl_data.cpp
  src/stl_format.cpp
  src/stl_to_grid.cpp
  src/wind_converter.cpp
)

target_include_directories(preprocessing PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

target_link_libraries(preprocessing
  gaden_common::gaden_common
  PNG::PNG
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

# OccupancyGrid converter
add_executable(occupancy_grid_converter
  src/main_occupancy_converter.cpp
)

target_include_directories(occupancy_grid_converter PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

target_link_libraries(occupancy_grid_converter
  gaden_common::gaden_common
)

ament_target_dependencies(occupancy_grid_converter
  gaden_common
)

install(TARGETS occupancy_grid_converter
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
