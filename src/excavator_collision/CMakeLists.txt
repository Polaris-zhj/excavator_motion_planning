cmake_minimum_required(VERSION 3.8)
project(excavator_collision)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fcl REQUIRED)
find_package(urdf REQUIRED)
find_package(excavator_common REQUIRED)
find_package(excavator_interfaces REQUIRED)

add_library(excavator_collision src/collision_checker.cpp)
target_include_directories(excavator_collision PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(excavator_collision fcl Eigen3::Eigen)
ament_target_dependencies(excavator_collision
  ament_index_cpp urdf excavator_common excavator_interfaces
)
target_compile_options(excavator_collision PRIVATE -Wall -Wextra -Wpedantic)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(collision_checker_test test/collision_checker_test.cpp)
  target_compile_definitions(collision_checker_test PRIVATE
    TEST_MESH_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/tetrahedron.stl")
  target_link_libraries(collision_checker_test excavator_collision)
endif()

install(TARGETS excavator_collision EXPORT excavator_collisionTargets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(excavator_collisionTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(ament_index_cpp Eigen3 fcl urdf excavator_common
  excavator_interfaces)
ament_package()
