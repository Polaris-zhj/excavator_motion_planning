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
find_package(excavator_common REQUIRED)

add_library(kinematics
  src/urdf_kinematics_model.cpp
  src/workspace_checker.cpp
)
target_include_directories(kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(kinematics Eigen3::Eigen)
ament_target_dependencies(kinematics urdf excavator_common)
target_compile_options(kinematics PRIVATE -Wall -Wextra -Wpedantic)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(urdf_kinematics_test test/urdf_kinematics_test.cpp)
  target_compile_definitions(urdf_kinematics_test PRIVATE
    TEST_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/planar_robot.urdf")
  target_link_libraries(urdf_kinematics_test kinematics)
  ament_add_gtest(workspace_checker_test test/workspace_checker_test.cpp)
  target_link_libraries(workspace_checker_test kinematics)
endif()

install(TARGETS kinematics EXPORT kinematicsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(kinematicsTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 urdf excavator_common)
ament_package()
