cmake_minimum_required(VERSION 3.8)
project(excavator_visualization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_AUTOMOC ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(excavator_common REQUIRED)
find_package(excavator_interfaces REQUIRED)
find_package(kinematics REQUIRED)
find_package(urdf REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(excavator_visualization_core
  src/planning_scene_manager.cpp
  src/replay_manager.cpp
  src/trajectory_occupancy_visualizer.cpp
  src/trajectory_plot_data.cpp
  src/trajectory_visualizer.cpp
)
target_include_directories(excavator_visualization_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(excavator_visualization_core
  geometry_msgs nav_msgs shape_msgs visualization_msgs excavator_interfaces
  excavator_common)
target_link_libraries(excavator_visualization_core kinematics::kinematics)
target_compile_options(excavator_visualization_core PRIVATE
  -Wall -Wextra -Wpedantic)
set_target_properties(excavator_visualization_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON)

add_executable(trajectory_visualizer_node
  src/trajectory_visualizer_node.cpp)
ament_target_dependencies(trajectory_visualizer_node
  rclcpp builtin_interfaces sensor_msgs trajectory_msgs nav_msgs urdf
  visualization_msgs excavator_common excavator_interfaces)
target_link_libraries(trajectory_visualizer_node
  excavator_visualization_core kinematics::kinematics)
target_compile_options(trajectory_visualizer_node PRIVATE
  -Wall -Wextra -Wpedantic)

add_executable(trajectory_replay_node src/trajectory_replay_node.cpp)
ament_target_dependencies(trajectory_replay_node
  rclcpp builtin_interfaces sensor_msgs trajectory_msgs excavator_common
  excavator_interfaces)
target_link_libraries(trajectory_replay_node excavator_visualization_core)
target_compile_options(trajectory_replay_node PRIVATE
  -Wall -Wextra -Wpedantic)

add_executable(target_marker_node src/target_marker_node.cpp)
ament_target_dependencies(target_marker_node
  rclcpp geometry_msgs sensor_msgs visualization_msgs interactive_markers
  excavator_common)
target_link_libraries(target_marker_node kinematics::kinematics)
target_compile_options(target_marker_node PRIVATE -Wall -Wextra -Wpedantic)

add_library(excavator_visualization_panels SHARED
  rviz_plugins/joint_plot_panel.cpp
  rviz_plugins/planning_panel.cpp
)
target_include_directories(excavator_visualization_panels PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/rviz_plugins)
ament_target_dependencies(excavator_visualization_panels
  rclcpp rclcpp_action rviz_common pluginlib interactive_markers sensor_msgs
  std_msgs trajectory_msgs visualization_msgs excavator_interfaces)
target_link_libraries(excavator_visualization_panels
  excavator_visualization_core Qt5::Widgets rviz_common::rviz_common)
target_compile_options(excavator_visualization_panels PRIVATE
  -Wall -Wextra -Wpedantic)
pluginlib_export_plugin_description_file(rviz_common
  rviz_plugins/plugin_description.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(trajectory_plot_data_test test/trajectory_plot_data_test.cpp)
  target_link_libraries(trajectory_plot_data_test
    excavator_visualization_core)

  ament_add_gtest(planning_scene_manager_test
    test/planning_scene_manager_test.cpp)
  target_link_libraries(planning_scene_manager_test
    excavator_visualization_core)

  ament_add_gtest(replay_manager_test test/replay_manager_test.cpp)
  target_link_libraries(replay_manager_test excavator_visualization_core)

  ament_add_gtest(trajectory_visualizer_test
    test/trajectory_visualizer_test.cpp)
  target_compile_definitions(trajectory_visualizer_test PRIVATE
    TEST_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/planar_robot.urdf")
  target_link_libraries(trajectory_visualizer_test
    excavator_visualization_core kinematics::kinematics)
endif()

install(TARGETS excavator_visualization_core
  EXPORT excavator_visualizationTargets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS trajectory_visualizer_node trajectory_replay_node
  target_marker_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS excavator_visualization_panels LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_targets(excavator_visualizationTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs nav_msgs shape_msgs visualization_msgs
  excavator_interfaces excavator_common kinematics)
ament_package()
