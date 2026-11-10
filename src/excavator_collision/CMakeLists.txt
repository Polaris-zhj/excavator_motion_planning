cmake_minimum_required(VERSION 3.8)
project(excavator_collision)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(excavator_common REQUIRED)
find_package(excavator_interfaces REQUIRED)

# Vendored FCL 0.7.0 subset (third_library/fcl). The
# target deliberately has a different name from the system FCL package and is
# linked privately, so no FCL headers or types escape this package.
if(NOT TARGET excavator_fcl)
  add_subdirectory(
    "${CMAKE_CURRENT_SOURCE_DIR}/third_library/fcl"
    "${CMAKE_CURRENT_BINARY_DIR}/fcl_vendor")
endif()

# Shared library: the vendored FCL static library is linked privately and its
# symbols stay inside this library, so consumers do not need FCL.
add_library(excavator_collision SHARED
  src/fcl_collision_checker.cpp
  src/robot_collision_geometry_model.cpp
  src/fcl_collision_model_builder.cpp
  src/collision_checker_factory.cpp
  src/obstacle_adapter.cpp)

target_include_directories(excavator_collision
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Keep the vendored FCL private: it must not become part of the exported
# interface. Explicit imported targets are used instead of
# ament_target_dependencies because the latter uses the plain
# target_link_libraries signature, which cannot be mixed with keyword usage.
target_link_libraries(excavator_collision
  PRIVATE
    excavator_fcl
    Eigen3::Eigen
    yaml-cpp
    ament_index_cpp::ament_index_cpp
  PUBLIC
    ${geometry_msgs_TARGETS}
    ${shape_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    excavator_common::excavator_common
    ${excavator_interfaces_TARGETS})

target_compile_options(excavator_collision PRIVATE
  -Wall -Wextra -Wpedantic)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(collision_checker_test test/collision_checker_test.cpp)
  target_compile_definitions(collision_checker_test PRIVATE
    TEST_MESH_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/tetrahedron.stl"
    TEST_DH_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../excavator_description/config/excavator_dh.yaml"
    TEST_COLLISION_YAML_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../excavator_description/config/excavator_collision.yaml")
  ament_target_dependencies(collision_checker_test
    excavator_common excavator_interfaces)
  target_link_libraries(collision_checker_test excavator_collision)

  ament_add_gtest(collision_checker_factory_test
    test/collision_checker_factory_test.cpp)
  ament_target_dependencies(collision_checker_factory_test
    excavator_common excavator_interfaces)
  target_link_libraries(collision_checker_factory_test excavator_collision)

  ament_add_gtest(collision_benchmark_test
    test/collision_benchmark_test.cpp)
  target_compile_definitions(collision_benchmark_test PRIVATE
    TEST_DH_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../excavator_description/config/excavator_dh.yaml"
    TEST_COLLISION_YAML_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../excavator_description/config/excavator_collision.yaml")
  ament_target_dependencies(collision_benchmark_test
    excavator_common excavator_interfaces)
  target_link_libraries(collision_benchmark_test excavator_collision)
endif()

install(TARGETS excavator_collision EXPORT excavator_collisionTargets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(excavator_collisionTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(ament_index_cpp Eigen3 geometry_msgs shape_msgs
  excavator_common excavator_interfaces)
ament_package()
