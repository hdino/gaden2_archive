cmake_minimum_required(VERSION 3.5)
project(gaden_filament_simulator)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common_config.cmake)

# find dependencies
find_package(gaden_common REQUIRED)

add_executable(filament_simulator
  src/filament.cpp
  src/filament_simulator.cpp
  src/main.cpp
)

target_include_directories(filament_simulator PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

target_link_libraries(filament_simulator
  gaden_common::gaden_common
)

ament_target_dependencies(filament_simulator
  gaden_common
)

install(TARGETS filament_simulator
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
