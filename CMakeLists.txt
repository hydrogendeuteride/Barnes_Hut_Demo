cmake_minimum_required(VERSION 3.24)
project(Barnes_Hut_Demo)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -mavx2 -fopenmp -march=native -mfma -funroll-loops -fprefetch-loop-arrays")
find_package(Eigen3 3.3 NO_MODULE)
find_package(SFML 2.5.1 REQUIRED network audio graphics window system)

add_executable(Barnes_Hut_Demo main.cc quadtree.h quadtree.cc barnes_hut.h numerics.h acceleration.h barnes_hut.cc Drawable.h constants.h constants.h distribution.h physics.h)

if(SFML_FOUND)
    include_directories(${SFML_INCLUDE_DIR})
    target_link_libraries(Barnes_Hut_Demo PRIVATE sfml-system sfml-graphics sfml-window)
endif()