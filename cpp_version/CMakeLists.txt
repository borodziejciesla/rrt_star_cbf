cmake_minimum_required(VERSION 3.16)
project(safe_cbf_rrt LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(SAFE_CBF_RRT_BUILD_PYTHON "Build pybind11 module" ON)
option(SAFE_CBF_RRT_BUILD_TESTS "Build gtest unit tests" ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(safe_cbf_rrt STATIC
  src/grid.cpp
  src/poisson_safety.cpp
  src/planner.cpp
)
target_include_directories(safe_cbf_rrt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(safe_cbf_rrt PUBLIC Eigen3::Eigen)
target_compile_options(safe_cbf_rrt PRIVATE -Wall -Wextra)

if(SAFE_CBF_RRT_BUILD_PYTHON)
  find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
  find_package(pybind11 CONFIG REQUIRED)
  pybind11_add_module(safe_cbf_rrt_py python/bindings.cpp)
  target_link_libraries(safe_cbf_rrt_py PRIVATE safe_cbf_rrt)
endif()

if(SAFE_CBF_RRT_BUILD_TESTS)
  find_package(GTest REQUIRED)
  enable_testing()
  add_executable(test_safe_cbf_rrt tests/test_safe_cbf_rrt.cpp)
  target_link_libraries(test_safe_cbf_rrt PRIVATE safe_cbf_rrt GTest::gtest GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(test_safe_cbf_rrt)
endif()
