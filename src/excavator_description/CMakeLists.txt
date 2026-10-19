cmake_minimum_required(VERSION 3.8)
project(excavator_description)

find_package(ament_cmake REQUIRED)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

install(
  DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
