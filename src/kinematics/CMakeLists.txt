cmake_minimum_required(VERSION 3.8)
project(kinematics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(excavator_common REQUIRED)

add_library(kinematics INTERFACE)
target_include_directories(kinematics INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(kinematics INTERFACE excavator_common)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS kinematics EXPORT kinematicsTargets)
ament_export_targets(kinematicsTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(excavator_common)
ament_package()
