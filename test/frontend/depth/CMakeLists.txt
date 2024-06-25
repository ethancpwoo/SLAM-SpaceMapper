set(CMAKE_CXX_FLAGS "-std=c++11")
cmake_minimum_required(VERSION 3.7...3.29)

if(${CMAKE_VERSION} VERSION_LESS 3.12)
    cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
endif()
project(triangulation)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse4.1") 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=nehalem")  
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse4.1 -msse4.2")  
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")  
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse3")  

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(triangulation triangulation.cpp)

target_link_libraries(triangulation ${OpenCV_LIBS})
