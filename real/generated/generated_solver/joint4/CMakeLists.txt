cmake_minimum_required(VERSION 3.15)
project(osqp_ext_emosqp)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)  # -fPIC
set(OSQP_EMBEDDED_MODE "1")

file(
  GLOB
  OSQP_SOURCES
  src/*.c)

add_library(osqpstatic
  STATIC
    ${OSQP_SOURCES}
  )

target_include_directories(osqpstatic PUBLIC inc/public inc/private .)
target_include_directories(osqpstatic PRIVATE inc/private .)

include(FetchContent)
FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git)
FetchContent_MakeAvailable(pybind11)

file(
  GLOB
  EXT_SOURCES
  mpc_j4workspace.c
  mpc_j4workspace.h
  bindings.cpp)

pybind11_add_module(emosqp ${OSQP_SOURCES} ${EXT_SOURCES})
target_link_libraries(emosqp PUBLIC pybind11::module osqpstatic)