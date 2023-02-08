cmake_minimum_required(VERSION 3.24)
project(Barnes_Hut_Demo)

set(CMAKE_CXX_STANDARD 23)
find_package(Eigen3 3.3 NO_MODULE)

add_executable(Barnes_Hut_Demo main.cc quadtree.h quadtree.cc barnes_hut.h numerics.h acceleration.h barnes_hut.cc)
