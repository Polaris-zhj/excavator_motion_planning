cmake_minimum_required(VERSION 3.8)
project(kinematics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdf REQUIRED)

add_library(kinematics
  src/urdf_kinematics_model.cpp
)
target_include_directories(kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(kinematics PUBLIC Eigen3::Eigen)
ament_target_dependencies(kinematics urdf)
target_compile_options(kinematics PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS kinematics EXPORT kinematicsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(kinematicsTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 urdf)
ament_package()
