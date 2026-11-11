cmake_minimum_required(VERSION 3.8)
project(excavator_kinematics)

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

# Vendored Orocos KDL 1.5.4 is a private implementation dependency.
set(KDL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_library/KDL")
add_library(excavator_kdl STATIC
  ${KDL_DIR}/frames.cpp
  ${KDL_DIR}/joint.cpp
  ${KDL_DIR}/segment.cpp
  ${KDL_DIR}/chain.cpp
  ${KDL_DIR}/jntarray.cpp
  ${KDL_DIR}/jacobian.cpp
  ${KDL_DIR}/rotationalinertia.cpp
  ${KDL_DIR}/rigidbodyinertia.cpp
  ${KDL_DIR}/framevel.cpp
  ${KDL_DIR}/frameacc.cpp
  ${KDL_DIR}/jntarrayvel.cpp
  ${KDL_DIR}/jntarrayacc.cpp
  ${KDL_DIR}/chainfksolverpos_recursive.cpp
  ${KDL_DIR}/chainjnttojacsolver.cpp
  ${KDL_DIR}/chainiksolvervel_wdls.cpp
  ${KDL_DIR}/chainiksolverpos_nr_jl.cpp
  ${KDL_DIR}/utilities/utility.cxx
  ${KDL_DIR}/utilities/utility_io.cxx
  ${KDL_DIR}/utilities/error_stack.cxx
  ${KDL_DIR}/utilities/svd_eigen_HH.cpp)
target_include_directories(excavator_kdl
  PUBLIC ${KDL_DIR})
target_link_libraries(excavator_kdl PUBLIC Eigen3::Eigen)
set_target_properties(excavator_kdl PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON)
target_compile_options(excavator_kdl PRIVATE
  -w -fvisibility=hidden -fvisibility-inlines-hidden)

add_library(excavator_kinematics SHARED
  kinematics/kinematics_model.cpp
  kinematics/kinematics_solver_factory.cpp
  model_converter/model_config_parser.cpp
  model_converter/urdf_model_converter.cpp
  model_converter/dh_model_converter.cpp
  backend/kdl/kdl_model_converter.cpp
  backend/kdl/kdl_type_converter.cpp
  backend/kdl/kdl_solver_bundle.cpp
  backend/kdl/kdl_kinematics_solver.cpp)

target_include_directories(excavator_kinematics
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${KDL_DIR})
target_link_libraries(excavator_kinematics
  PRIVATE excavator_kdl Eigen3::Eigen yaml-cpp)
ament_target_dependencies(excavator_kinematics urdf)
target_compile_options(excavator_kinematics PRIVATE
  -Wall -Wextra -Wpedantic)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(kinematics_model_test test/kinematics_model_test.cpp)
  target_link_libraries(kinematics_model_test excavator_kinematics)

  ament_add_gtest(kinematics_converter_test test/kinematics_converter_test.cpp)
  target_link_libraries(kinematics_converter_test
    excavator_kinematics yaml-cpp)
  ament_target_dependencies(kinematics_converter_test urdf)

  ament_add_gtest(kdl_kinematics_test test/kdl_kinematics_test.cpp)
  target_link_libraries(kdl_kinematics_test excavator_kinematics)
endif()

install(TARGETS excavator_kinematics EXPORT excavator_kinematicsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_export_targets(excavator_kinematicsTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 urdf yaml_cpp_vendor)
ament_package()
