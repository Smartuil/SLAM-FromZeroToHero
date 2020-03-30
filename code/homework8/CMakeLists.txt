cmake_minimum_required(VERSION 3.15)
project(homework8)

set(CMAKE_CXX_STANDARD 14)

include_directories("/usr/local/include/eigen3")
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})


add_executable(homework8 draw_trajectory.cpp)
target_link_libraries(homework8 ${Pangolin_LIBRARIES})