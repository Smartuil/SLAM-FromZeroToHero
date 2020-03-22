cmake_minimum_required(VERSION 3.15)
project(homework3)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
include_directories(
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(1 1.cpp)
target_link_libraries(1 ${OpenCV_LIBS})

add_executable(2 2.cpp)
target_link_libraries(2 ${OpenCV_LIBS})