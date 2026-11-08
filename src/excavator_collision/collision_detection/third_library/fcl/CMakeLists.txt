cmake_minimum_required(VERSION 3.14)
project(excavator_fcl VERSION 0.7.0 LANGUAGES CXX)

# Local FCL subset used by excavator_collision.
#
# The upstream FCL source tree (https://github.com/flexible-collision-library/fcl,
# release 0.7.0) is vendored here so the planning stack does not depend on a
# system libfcl installation. Only the core geometry/broadphase/narrowphase
# code is built; the optional OctoMap backend is disabled.
#
# This target is intentionally named excavator_fcl instead of fcl to avoid
# clashing with an installed system FCL package.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.0.5 REQUIRED)
find_package(PkgConfig REQUIRED)
# FCL 0.7 requires libccd for its GJK/EPA convex algorithms.
pkg_search_module(CCD REQUIRED IMPORTED_TARGET ccd libccd)

set(FCL_VERSION "0.7.0")
set(FCL_MAJOR_VERSION 0)
set(FCL_MINOR_VERSION 7)
set(FCL_PATCH_VERSION 0)
# The local build deliberately excludes OctoMap and SSE-specific code.
set(FCL_HAVE_SSE 0)
set(FCL_HAVE_OCTOMAP 0)
set(FCL_ENABLE_PROFILING 0)
set(GENERATED_FILE_MARKER "GENERATED FILE DO NOT EDIT")

set(FCL_SOURCE_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(FCL_BINARY_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/include")

configure_file(
  "${FCL_SOURCE_INCLUDE_DIR}/fcl/config.h.in"
  "${FCL_BINARY_INCLUDE_DIR}/fcl/config.h"
  @ONLY)

# OctoMap support is disabled, so its implementation file is not compiled.
file(GLOB_RECURSE FCL_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
list(FILTER FCL_SOURCES EXCLUDE REGEX "/octree/")

add_library(excavator_fcl STATIC ${FCL_SOURCES})

include(GenerateExportHeader)
generate_export_header(excavator_fcl
  BASE_NAME FCL
  EXPORT_FILE_NAME "${FCL_BINARY_INCLUDE_DIR}/fcl/export.h")

# Build the umbrella header exactly like upstream FCL: all public non-detail
# headers, plus the generated config/export headers.
file(GLOB_RECURSE FCL_SOURCE_HEADERS RELATIVE "${FCL_SOURCE_INCLUDE_DIR}"
  "${FCL_SOURCE_INCLUDE_DIR}/fcl/*.h")
set(FCL_GENERATED_HEADERS "fcl/config.h" "fcl/export.h")
set(FCL_ALL_HEADERS ${FCL_GENERATED_HEADERS} ${FCL_SOURCE_HEADERS})

set(INCLUDE_DIRECTIVES)
foreach(HEADER ${FCL_ALL_HEADERS})
  if(NOT HEADER MATCHES "(/detail/|-inl.h$)")
    set(INCLUDE_DIRECTIVES
      "${INCLUDE_DIRECTIVES}\n#include \"${HEADER}\"")
  endif()
endforeach()
string(STRIP "${INCLUDE_DIRECTIVES}" INCLUDE_DIRECTIVES)

configure_file(
  "${FCL_SOURCE_INCLUDE_DIR}/fcl/fcl.h.in"
  "${FCL_BINARY_INCLUDE_DIR}/fcl/fcl.h"
  @ONLY)

set_target_properties(excavator_fcl PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_compile_features(excavator_fcl PUBLIC cxx_std_17)
target_include_directories(excavator_fcl PUBLIC
  $<BUILD_INTERFACE:${FCL_BINARY_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${FCL_SOURCE_INCLUDE_DIR}>)
target_link_libraries(excavator_fcl PUBLIC
  Eigen3::Eigen
  PkgConfig::CCD)

# The vendored library is linked into libexcavator_collision.so. Hide its
# symbols so they cannot clash with a system libfcl loaded by another
# component in the same process.
target_compile_definitions(excavator_fcl PUBLIC FCL_STATIC_DEFINE)
target_compile_options(excavator_fcl PRIVATE
  -Wall -Wextra -fvisibility=hidden -fvisibility-inlines-hidden)

# Hidden upstream test/doc/install machinery is intentionally omitted; the
# consumer builds this target as a private static dependency.
