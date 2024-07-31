cmake_minimum_required(VERSION 3.22.1)
project(PathTracer_CPP)

# C++23
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# find vulkan
set(VULKAN_SDK "C:/VulkanSDK/1.3.290.0")  # Adjust the path to your Vulkan SDK version
set(Vulkan_INCLUDE_DIR "${VULKAN_SDK}/Include")
set(Vulkan_LIBRARY "${VULKAN_SDK}/Lib/vulkan-1.lib")
include_directories(${Vulkan_INCLUDE_DIR})

# compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
message(STATUS "Using CXX Flags: ${CMAKE_CXX_FLAGS} -O3 -march=native -std=c++23")

# Add GLFW
set(GLFW_DIR ${CMAKE_SOURCE_DIR}/Libraries/glfw-3.4)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_USE_WAYLAND OFF CACHE BOOL "" FORCE)  # Disable Wayland
add_subdirectory(${GLFW_DIR})

# Add GLM
set(GLM_DIR ${CMAKE_SOURCE_DIR}/libraries/glm) # Adjust this path if necessary
include_directories(${GLM_DIR})

# source files
include_directories(include) # include the header directory
add_executable(PathTracer_CPP src/main.cpp #include all source and header files
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
        src/Config.cpp
)

# Link Vulkan library
target_link_libraries(PathTracer_CPP ${Vulkan_LIBRARY})