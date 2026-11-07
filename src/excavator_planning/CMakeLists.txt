cmake_minimum_required(VERSION 3.8)
project(excavator_planning)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(excavator_interfaces REQUIRED)
find_package(excavator_common REQUIRED)
find_package(excavator_collision REQUIRED)
find_package(pluginlib REQUIRED)
find_package(toppra REQUIRED)
find_package(g3log CONFIG REQUIRED)
# g3sinks installs no g3logrotateConfig.cmake, so locate the LogRotate sink
# (headers + library) directly instead of using find_package().
find_path(G3LOGROTATE_INCLUDE_DIR NAMES g3sinks/LogRotate.h)
find_library(G3LOGROTATE_LIBRARY NAMES g3logrotate)
if(NOT G3LOGROTATE_INCLUDE_DIR OR NOT G3LOGROTATE_LIBRARY)
  message(FATAL_ERROR
    "g3logrotate not found: install the g3sinks LogRotate sink")
endif()
add_definitions(-DG3_DYNAMIC_LOGGING)

add_library(excavator_planning
  src/baseline_planners.cpp
  src/path_post_processor.cpp
  src/planner_factory.cpp
  src/time_parameterizer_factory.cpp
  src/toppra_time_parameterizer.cpp)
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
target_link_libraries(planning_server excavator_planning
  excavator_common::excavator_common excavator_collision::excavator_collision)
target_compile_options(planning_server PRIVATE -Wall -Wextra -Wpedantic)

add_executable(toppra_parameterize_csv src/toppra_parameterize_csv.cpp)
target_link_libraries(toppra_parameterize_csv excavator_planning toppra::toppra)
target_compile_options(toppra_parameterize_csv PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(excavator_planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(excavator_planning excavator_common)
target_link_libraries(excavator_planning toppra::toppra)
target_link_libraries(excavator_planning g3log g3logrotate)
target_compile_options(excavator_planning PRIVATE -Wall -Wextra -Wpedantic)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(baseline_planners_test test/baseline_planners_test.cpp)
  target_link_libraries(baseline_planners_test excavator_planning)
  ament_add_gtest(time_parameterizer_test test/time_parameterizer_test.cpp)
  target_link_libraries(time_parameterizer_test excavator_planning)
endif()

install(TARGETS planning_server toppra_parameterize_csv
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS excavator_planning_plugins LIBRARY DESTINATION lib)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(TARGETS excavator_planning EXPORT excavator_planningTargets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(excavator_planningTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(excavator_common excavator_collision toppra)
ament_package()
