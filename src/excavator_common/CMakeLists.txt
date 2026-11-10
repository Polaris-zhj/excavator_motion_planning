cmake_minimum_required(VERSION 3.8)
project(excavator_common)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdf REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(excavator_common
  src/dh_kinematics_model.cpp
  src/dh_parameters.cpp
  src/linear_trajectory_interpolator.cpp
  src/urdf_kinematics_model.cpp
  src/workspace_checker.cpp
)
target_include_directories(excavator_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(excavator_common
  PUBLIC Eigen3::Eigen yaml-cpp urdf::urdf)
target_compile_options(excavator_common PRIVATE
  -Wall -Wextra -Wpedantic)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(joint_trajectory_test test/joint_trajectory_test.cpp)
  target_link_libraries(joint_trajectory_test excavator_common)

  ament_add_gtest(linear_trajectory_interpolator_test
    test/linear_trajectory_interpolator_test.cpp)
  target_link_libraries(linear_trajectory_interpolator_test excavator_common)

  ament_add_gtest(urdf_kinematics_test test/urdf_kinematics_test.cpp)
  target_compile_definitions(urdf_kinematics_test PRIVATE
    TEST_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/planar_robot.urdf")
  target_link_libraries(urdf_kinematics_test excavator_common)

  ament_add_gtest(dh_kinematics_test test/dh_kinematics_test.cpp)
  target_compile_definitions(dh_kinematics_test PRIVATE
    TEST_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/excavator_4r.urdf"
    TEST_DH_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../excavator_description/config/excavator_dh.yaml")
  target_link_libraries(dh_kinematics_test excavator_common)

  ament_add_gtest(workspace_checker_test test/workspace_checker_test.cpp)
  target_link_libraries(workspace_checker_test excavator_common)
endif()

install(TARGETS excavator_common EXPORT excavator_commonTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY data/ DESTINATION include/data
  FILES_MATCHING PATTERN "*.h")
ament_export_targets(excavator_commonTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 urdf yaml-cpp)
ament_package()
