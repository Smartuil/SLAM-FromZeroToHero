cmake_minimum_required(VERSION 3.15)
project(homework5)

set(CMAKE_CXX_STANDARD 14)

include_directories("/usr/local/include/eigen3")

add_executable(homework5 rotationTransforms.cpp)