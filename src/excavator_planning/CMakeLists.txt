cmake_minimum_required(VERSION 3.8)
project(excavator_planning)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(excavator_interfaces REQUIRED)
find_package(kinematics REQUIRED)
find_package(excavator_common REQUIRED)
find_package(excavator_collision REQUIRED)
find_package(pluginlib REQUIRED)


add_library(excavator_planning
  src/baseline_planners.cpp
  src/path_post_processor.cpp
  src/planner_factory.cpp)
set_target_properties(excavator_planning PROPERTIES
  POSITION_INDEPENDENT_CODE ON)

add_library(excavator_planning_plugins SHARED
  src/plugins/linear_planner_plugin.cpp)
ament_target_dependencies(excavator_planning_plugins excavator_common pluginlib)
target_link_libraries(excavator_planning_plugins excavator_planning)
pluginlib_export_plugin_description_file(excavator_planning planner_plugins.xml)

add_executable(planning_server src/planning_server.cpp)
ament_target_dependencies(planning_server rclcpp rclcpp_action builtin_interfaces
  sensor_msgs shape_msgs trajectory_msgs visualization_msgs pluginlib
  excavator_interfaces excavator_collision)
target_link_libraries(planning_server excavator_planning kinematics::kinematics
  excavator_collision::excavator_collision)
target_compile_options(planning_server PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(excavator_planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(excavator_planning excavator_common)
target_compile_options(excavator_planning PRIVATE -Wall -Wextra -Wpedantic)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(baseline_planners_test test/baseline_planners_test.cpp)
  target_link_libraries(baseline_planners_test excavator_planning)
endif()

install(TARGETS planning_server DESTINATION lib/${PROJECT_NAME})
install(TARGETS excavator_planning_plugins LIBRARY DESTINATION lib)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(TARGETS excavator_planning EXPORT excavator_planningTargets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(excavator_planningTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(excavator_common excavator_collision)
ament_package()
