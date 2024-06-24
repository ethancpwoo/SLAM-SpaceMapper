set(CMAKE_CXX_FLAGS "-std=c++11")
cmake_minimum_required(VERSION 3.7...3.29)

if(${CMAKE_VERSION} VERSION_LESS 3.12)
    cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
endif()
project(frontEnd)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(frontend frontend.cpp)

target_link_libraries(frontend ${OpenCV_LIBS})
