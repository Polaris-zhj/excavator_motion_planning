cmake_minimum_required(VERSION 3.16)
project(toppra VERSION 0.6.10 LANGUAGES CXX)



find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(g3log CONFIG REQUIRED)
# g3sinks installs no g3logrotateConfig.cmake, so locate the LogRotate sink
# (headers + library) directly instead of using find_package().
find_path(G3LOGROTATE_INCLUDE_DIR NAMES g3sinks/LogRotate.h)
find_library(G3LOGROTATE_LIBRARY NAMES g3logrotate)
if(NOT G3LOGROTATE_INCLUDE_DIR OR NOT G3LOGROTATE_LIBRARY)
  message(FATAL_ERROR
    "g3logrotate not found: install the g3sinks LogRotate sink")
endif()




# Only the core algorithm and its dependency-free Seidel solver are built.
# Optional dependencies (qpOASES, GLPK, pinocchio, msgpack) are intentionally
# left disabled: Seidel is sufficient here and avoids the GPL GLPK dependency.
add_library(toppra STATIC
  src/algorithm.cpp
  algorithm/toppra.cpp
  src/constraint.cpp
  constraint/cartesian_velocity_norm.cpp
  constraint/joint_torque.cpp
  constraint/linear_joint_acceleration.cpp
  constraint/linear_joint_velocity.cpp
  src/geometric_path.cpp
  geometric_path/piecewise_poly_path.cpp
  src/logging.cpp
  src/parametrizer.cpp
  parametrizer/const_accel.cpp
  parametrizer/spline.cpp
  src/solver.cpp
  solver/glpk-wrapper.cpp
  solver/qpOASES-wrapper.cpp
  solver/seidel.cpp
)


target_compile_features(toppra PUBLIC cxx_std_17)
set_target_properties(toppra PROPERTIES POSITION_INDEPENDENT_CODE ON)
# colcon does not set CMAKE_BUILD_TYPE by default; keep the solver fast anyway.
if(NOT CMAKE_BUILD_TYPE)
  target_compile_options(toppra PRIVATE -O2)
endif()

target_include_directories(toppra PUBLIC
  # Include root for both <toppra/...> subdirectory headers and the
  # "toppra/src/..." top-level headers.
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(toppra PUBLIC Eigen3::Eigen)
target_link_libraries(toppra PRIVATE g3log g3logrotate)

install(TARGETS toppra EXPORT toppraTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/"
  DESTINATION include/toppra
  FILES_MATCHING PATTERN "*.hpp")
install(FILES LICENSE
  DESTINATION share/${PROJECT_NAME})

ament_export_targets(toppraTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3)
ament_package()
