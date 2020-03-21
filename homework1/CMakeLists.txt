cmake_minimum_required(VERSION 3.15)
project(homework1)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(homework1 main.cpp)
target_link_libraries(homework1 ${OpenCV_LIBS})