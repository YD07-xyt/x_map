cmake_minimum_required(VERSION 3.10)
project(x_map)

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(x_map_node ros2/ros2.cpp)

add_executable(x_map main.cpp)
target_include_directories(x_map PRIVATE include)


ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  config
  launch
)