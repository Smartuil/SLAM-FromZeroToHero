cmake_minimum_required(VERSION 3.15)
project(homework2)

set(CMAKE_CXX_STANDARD 14)

include_directories( "/usr/local/include/eigen3" )

add_executable(1 1.cpp)
add_executable(2 2.cpp)
add_executable(3 3.cpp)