cmake_minimum_required(VERSION 3.5)
project(test_env2)

# find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
