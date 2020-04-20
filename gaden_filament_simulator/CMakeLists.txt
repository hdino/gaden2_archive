cmake_minimum_required(VERSION 3.5)
project(gaden_filament_simulator)

include(${CMAKE_CURRENT_SOURCE_DIR}/../common_config.cmake)

# find dependencies
find_package(gaden_common REQUIRED)

add_executable(filament_simulator
  src/environment_model.cpp
  src/environment_visualisation.cpp
  src/farrells_wind_model.cpp
  src/farrells_wind_model_noise.cpp
  src/filament_model.cpp
  src/filament_model_rviz.cpp
  src/filament_simulator.cpp
  src/gas_dispersion_model.cpp
  src/inline_wind_model.cpp
  src/main.cpp
  src/occupancy.cpp
  src/openvdb_environment_model.cpp
  src/simulator.cpp
  src/simulator_config.cpp
  src/wind_model.cpp
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
