cmake_minimum_required(VERSION 3.8)
project(excavator_common)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)

add_library(excavator_common INTERFACE)
target_include_directories(excavator_common INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS excavator_common EXPORT excavator_commonTargets)
ament_export_targets(excavator_commonTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
