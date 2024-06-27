cmake_minimum_required(VERSION 3.7...3.29)
project(frontEnd)

set(CMAKE_CXX_FLAGS "-std=c++17")

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

set(THIRD_PARTY_LIBS ${OpenCV_LIBS})

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories("/usr/include/eigen3")
add_subdirectory(src)
add_subdirectory(app)
