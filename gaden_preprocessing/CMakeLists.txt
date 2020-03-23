cmake_minimum_required(VERSION 3.5)
project(gaden_preprocessing)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common_config.cmake)

# find dependencies
find_package(rclcpp REQUIRED)

add_executable(preprocessing
  src/main.cpp
  src/preprocessing.cpp
)

target_include_directories(preprocessing PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

#target_link_libraries(preprocessing
#  yaml-cpp
#)

ament_target_dependencies(preprocessing
  rclcpp
)

install(TARGETS preprocessing
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
