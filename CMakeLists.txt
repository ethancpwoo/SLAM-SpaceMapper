cmake_minimum_required(VERSION 3.18)
project(slamspacemapper)

set(CMAKE_CXX_FLAGS "-std=c++17")

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

find_package(fbow REQUIRED)
include_directories(${fbow_INCLUDE_DIRS})

find_package(g2o REQUIRED)
include_directories(${g2o_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

set(THIRD_PARTY_LIBS 
    ${fbow_LIBS}
    ${OpenCV_LIBS}
    ${OPENGL_LIBS}
    ${Pangolin_LIBS}
    ${Sophus_LIBS}
    g2o_core g2o_stuff g2o_types_sba
    )

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories("/usr/include/eigen3")
add_subdirectory(src)
add_subdirectory(app)
