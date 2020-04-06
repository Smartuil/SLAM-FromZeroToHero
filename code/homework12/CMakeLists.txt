cmake_minimum_required(VERSION 3.15)
project(homework12)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(homework12 virtual-billboard.cpp)
target_link_libraries( homework12 ${OpenCV_LIBS} )