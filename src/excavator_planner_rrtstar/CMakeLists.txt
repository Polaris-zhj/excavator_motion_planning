cmake_minimum_required(VERSION 3.8)
project(excavator_planner_rrtstar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(excavator_common REQUIRED)
find_package(excavator_planning REQUIRED)
find_package(pluginlib REQUIRED)

add_library(excavator_rrt_star_planner SHARED src/rrt_star_planner.cpp)
target_include_directories(excavator_rrt_star_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(excavator_rrt_star_planner
  excavator_common pluginlib)
target_link_libraries(excavator_rrt_star_planner
  excavator_planning::excavator_planning)
target_compile_options(excavator_rrt_star_planner PRIVATE
  -Wall -Wextra -Wpedantic)

# Register the plugin in the excavator_planning plugin category so the
# planning server's pluginlib loader can discover it without any code change.
pluginlib_export_plugin_description_file(excavator_planning planner_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(rrt_star_planner_test test/rrt_star_planner_test.cpp)
  target_link_libraries(rrt_star_planner_test excavator_rrt_star_planner)
endif()

install(TARGETS excavator_rrt_star_planner LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_dependencies(excavator_common excavator_planning pluginlib)
ament_package()
