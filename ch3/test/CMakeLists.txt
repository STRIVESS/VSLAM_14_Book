cmake_minimum_required(VERSION 2.8)
project(test)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3")

# 添加Eigen头文件
include_directories("/usr/local/include/eigen3")
add_executable(test_EigenMatrix test_EigenMatrix.cpp)
add_executable(test_EigenGeometry test_EigenGeometry.cpp)
add_executable(EigenGeometry EigenGeometry.cpp)

