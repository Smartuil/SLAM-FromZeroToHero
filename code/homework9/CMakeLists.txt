cmake_minimum_required(VERSION 3.15)
project(homework9)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)
# 添加头文件
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(homework9 undistortImage.cpp)
target_link_libraries(homework9 ${OpenCV_LIBS})