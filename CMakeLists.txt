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
        src/main.cpp
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
        src/BVHNode.cpp
        include/BVHNode.h
        src/Render.cpp
        include/Render.h
        include/Camera.h
        src/Camera.cpp)
