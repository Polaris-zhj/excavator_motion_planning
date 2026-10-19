cmake_minimum_required(VERSION 3.8)
project(excavator_sim_viz)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(excavator_common REQUIRED)
find_package(excavator_planning REQUIRED)
find_package(excavator_interfaces REQUIRED)
find_package(kinematics REQUIRED)
find_package(excavator_collision REQUIRED)

add_library(excavator_simulator
  src/kinematic_simulator.cpp
  src/trajectory_controller.cpp
  src/velocity_tracking_controller.cpp)
target_include_directories(excavator_simulator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(excavator_simulator PUBLIC excavator_common::excavator_common
  excavator_planning::excavator_planning)
target_compile_options(excavator_simulator PRIVATE -Wall -Wextra -Wpedantic)

add_executable(trajectory_controller_node src/trajectory_controller_node.cpp)
ament_target_dependencies(trajectory_controller_node rclcpp rclcpp_action
  std_msgs sensor_msgs trajectory_msgs excavator_interfaces)
target_link_libraries(trajectory_controller_node excavator_simulator
  kinematics::kinematics excavator_collision::excavator_collision)
target_compile_options(trajectory_controller_node PRIVATE -Wall -Wextra -Wpedantic)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(kinematic_simulator_test test/kinematic_simulator_test.cpp)
  target_link_libraries(kinematic_simulator_test excavator_simulator)
  ament_add_gtest(trajectory_controller_test test/trajectory_controller_test.cpp)
  target_link_libraries(trajectory_controller_test excavator_simulator)
endif()

install(TARGETS excavator_simulator EXPORT excavator_sim_vizTargets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(TARGETS trajectory_controller_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_targets(excavator_sim_vizTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(excavator_common excavator_planning)
ament_package()
