cmake_minimum_required(VERSION 3.22.1)
project(PathTracer_CPP)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
message(STATUS "Using CXX Flags: ${CMAKE_CXX_FLAGS} -O3 -march=native -std=c++23")

include_directories(include) # include the header directory

add_executable(PathTracer_CPP src/main.cpp
        src/Vector3.cpp
        include/Vector3.h
        include/Sphere.h
        src/Sphere.cpp
        include/SceneObject.h
        src/SceneObject.cpp
        include/Ray.h
        src/Ray.cpp
        src/AABCubeCenter.cpp
        include/AABCubeCenter.h
        include/AABCubeCenter.h
        include/AABCubeBounds.h
        src/AABCubeBounds.cpp
        src/BoundingBox.cpp
        include/BoundingBox.h
        include/BVHNode.h
        src/Render.cpp
        include/Render.h
        include/Camera.h
        src/Camera.cpp
        include/Config.h
        src/BVHNode.cpp
        src/Config.cpp
        src/Config.cpp)