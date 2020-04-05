cmake_minimum_required(VERSION 3.15)
project(homework11)

set(CMAKE_CXX_STANDARD 14)

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )


add_executable(homework11 epipolar.cpp)
target_link_libraries( homework11 ${OpenCV_LIBS} )