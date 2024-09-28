cmake_minimum_required(VERSION 3.24)
project(Barnes_Hut_Demo)

set(CMAKE_CXX_STANDARD 20)

include_directories(include)

if (WIN32)
    if(DEFINED ENV{VCPKG_ROOT})
        set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")
        list(APPEND CMAKE_PREFIX_PATH "$ENV{VCPKG_ROOT}/installed/x64-windows/share")
    else()
        message(FATAL_ERROR "Please define VCPKG_ROOT environment variable pointing to your vcpkg installation")
    endif()

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Zi /Od /Ob0 /RTC1")
    set(CMAKE_CXX_FLAGS_RELEASE "/O2 /arch:AVX2 /openmp /fp:fast /Ot /GL")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /LTCG")
else ()
    set(CMAKE_CXX_FLAGS "-Wall -Wextra")

    set(CMAKE_CXX_FLAGS_DEBUG "-g")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -mavx2 -fopenmp -march=native -mfma -funroll-loops -fprefetch-loop-arrays")
endif ()

find_package(Eigen3 3.3 NO_MODULE)
find_package(SFML 2.5.1 REQUIRED network audio graphics window system)

find_package(ImGui-SFML REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLU REQUIRED)

add_executable(Barnes_Hut_Demo
        main.cc
        src/quadtree.cc
        src/barnes_hut.cc
)

include_directories(${SFML_INCLUDE_DIR})
target_link_libraries(Barnes_Hut_Demo ${OPENGL_LIBRARIES} ${GLU_LIBRARIES} sfml-system sfml-graphics sfml-window sfml-audio ImGui-SFML::ImGui-SFML)


if (WIN32 AND DEFINED ENV{VCPKG_ROOT})
    add_custom_command(TARGET Barnes_Hut_Demo POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
            "$ENV{VCPKG_ROOT}/installed/x64-windows/bin"
            $<TARGET_FILE_DIR:Barnes_Hut_Demo>)
endif ()