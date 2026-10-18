cmake_minimum_required(VERSION 3.8)
project(excavator_sim_viz)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(excavator_common REQUIRED)

add_executable(trajectory_player src/trajectory_player.cpp)
ament_target_dependencies(trajectory_player rclcpp sensor_msgs trajectory_msgs)
target_link_libraries(trajectory_player excavator_common)
target_compile_options(trajectory_player PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS trajectory_player DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
