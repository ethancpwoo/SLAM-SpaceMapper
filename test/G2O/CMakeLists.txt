cmake_minimum_required(VERSION 3.7...3.29)
set(CMAKE_CXX_FLAGS "-std=c++17")

project(g2o_test)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

find_package(g2o REQUIRED)
include_directories(${g2o_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")

add_executable(g2o_test g2o.cpp)

target_link_libraries(g2o_test ${g2o_LIB_TYPE})
