cmake_minimum_required(VERSION 3.8)
project(excavator_planning)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(excavator_common REQUIRED)

add_library(excavator_planning src/baseline_planners.cpp)
target_include_directories(excavator_planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(excavator_planning PUBLIC excavator_common)
target_compile_options(excavator_planning PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS excavator_planning EXPORT excavator_planningTargets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(excavator_planningTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(excavator_common)
ament_package()
