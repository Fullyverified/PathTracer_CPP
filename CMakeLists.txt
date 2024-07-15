cmake_minimum_required(VERSION 3.28)
project(PathTracer_CPP)

set(CMAKE_CXX_STANDARD 23)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfma -mavx2 -O3 -march=native")
message(STATUS "Using CXX Flags: ${CMAKE_CXX_FLAGS}")

include_directories(include) # include the header directory

add_executable(PathTracer_CPP src/main.cpp
        src/main.cpp
        src/Vector3.cpp
        include/Vector3.h
        src/main.cpp)
