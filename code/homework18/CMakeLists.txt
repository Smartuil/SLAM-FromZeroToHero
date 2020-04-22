cmake_minimum_required( VERSION 2.8 )
project( directBA )

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake_modules/)

find_package(OpenCV REQUIRED)
message(STATUS "opencv found: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "opencv libs: ${OpenCV_LIBS}")

find_package(G2O REQUIRED)
message(STATUS "G2O found: ${G2O_INCLUDE_DIRS}")
file(GLOB G2O_LIB /usr/local/lib/*g2o*.so)
message(STATUS "G2O libs: ${G2O_LIB}")

find_package(Eigen3 REQUIRED)
message(STATUS "eigen3 found: ${EIGEN3_INCLUDE_DIRS}")
message(STATUS "eigen3 libs: ${EIGEN3_LIBRARIES}")

find_package(Pangolin REQUIRED)
message(STATUS "Pangolin found: ${Pangolin_INCLUDE_DIRS}")
message(STATUS "Pangolin libs: ${Pangolin_LIBRARIES}")

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
)

add_executable(directBA directBA.cpp)
target_link_libraries(directBA
        ${OpenCV_LIBS}
        ${G2O_LIB}
        ${Pangolin_LIBRARIES}
        )



