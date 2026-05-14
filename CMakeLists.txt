cmake_minimum_required(VERSION 3.10)
project(lazycbs)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED 1)

# Set C++ version.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED 1)

find_package(Boost 1.58 REQUIRED COMPONENTS program_options)

set(LAZYCBS_BUILD_VARIANT "lazycbs_mapf" CACHE STRING "Build variant: lazycbs_mapf, og, target, or confilct")
set_property(CACHE LAZYCBS_BUILD_VARIANT PROPERTY STRINGS lazycbs_mapf og target confilct)

if(LAZYCBS_BUILD_VARIANT STREQUAL "og")
  set(LAZYCBS_DEFAULT_TARGET_SYMMETRY 0)
  set(LAZYCBS_DEFAULT_CONFLICT_TIEBREAKER 0)
elseif(LAZYCBS_BUILD_VARIANT STREQUAL "target")
  set(LAZYCBS_DEFAULT_TARGET_SYMMETRY 1)
  set(LAZYCBS_DEFAULT_CONFLICT_TIEBREAKER 0)
elseif(LAZYCBS_BUILD_VARIANT STREQUAL "confilct")
  set(LAZYCBS_DEFAULT_TARGET_SYMMETRY 1)
  set(LAZYCBS_DEFAULT_CONFLICT_TIEBREAKER 1)
elseif(LAZYCBS_BUILD_VARIANT STREQUAL "lazycbs_mapf")
  set(LAZYCBS_DEFAULT_TARGET_SYMMETRY 1)
  set(LAZYCBS_DEFAULT_CONFLICT_TIEBREAKER 1)
else()
  message(FATAL_ERROR
    "Unknown LAZYCBS_BUILD_VARIANT='${LAZYCBS_BUILD_VARIANT}'. "
    "Expected lazycbs_mapf, og, target, or confilct.")
endif()

add_subdirectory(library/geas)
#add_subdirectory(library/yaml-cpp)

add_library(lazycbs INTERFACE)
target_include_directories(
  lazycbs
  INTERFACE 
  include 
  library/geas/include
  #library/yaml-cpp/include
)

file(GLOB LAZY_CBS_SOURCES "include/lazycbs/*")
file(GLOB GEAS_SOURCES "library/geas/*")

add_executable(
  ${LAZYCBS_BUILD_VARIANT}
  example/lazycbs_test.cpp
  ${LAZY_CBS_SOURCES} ${GEAS_SOURCES}
)
set_target_properties(
  ${LAZYCBS_BUILD_VARIANT}
  PROPERTIES
  OUTPUT_NAME ${LAZYCBS_BUILD_VARIANT}
)
target_compile_definitions(
  ${LAZYCBS_BUILD_VARIANT}
  PRIVATE
  LAZYCBS_DEFAULT_TARGET_SYMMETRY=${LAZYCBS_DEFAULT_TARGET_SYMMETRY}
  LAZYCBS_DEFAULT_CONFLICT_TIEBREAKER=${LAZYCBS_DEFAULT_CONFLICT_TIEBREAKER}
)
target_link_libraries(
  ${LAZYCBS_BUILD_VARIANT}
  lazycbs
  geas
  yaml-cpp
  ${Boost_LIBRARIES}
)
