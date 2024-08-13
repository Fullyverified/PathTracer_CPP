cmake_minimum_required(VERSION 3.22.1)
project(LittleVulkanEngine)

# C++ 23
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
message(STATUS "Using CXX Flags: ${CMAKE_CXX_FLAGS} -O3 -march=native -std=c++23")

if (WIN32)  # Windows environment (MinGW)
    set(VULKAN_SDK "C:/VulkanSDK/1.3.290.0")  # Adjust the path to your Vulkan SDK version
    set(Vulkan_INCLUDE_DIR "${VULKAN_SDK}/Include")
    set(Vulkan_LIBRARY "${VULKAN_SDK}/Lib/vulkan-1.lib")
    include_directories(${Vulkan_INCLUDE_DIR})
    # Add GLFW
    set(GLFW_DIR ${CMAKE_SOURCE_DIR}/libraries/glfw-3.4)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)  # disable Wayland
    set(GLFW_BUILD_X11 OFF CACHE BOOL "" FORCE)       # disable X11
    add_subdirectory(${GLFW_DIR})

    # Add GLM
    #set(GLM_DIR ${CMAKE_SOURCE_DIR}/libraries/glm) # Adjust this path if necessary
    #include_directories(${GLM_DIR})

    # Add TinyObjLoader
    set(TinyObjLoader_DIR ${CMAKE_SOURCE_DIR}/libraries/tinyobjloader) # Adjust this path if necessary
    include_directories(${TinyObjLoader_DIR})

elseif (UNIX)   # Linux environment (WSL)
    find_package(Vulkan REQUIRED)
    include_directories(${Vulkan_INCLUDE_DIRS})

    # Add GLFW using system packages
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(GLFW REQUIRED glfw3)
    include_directories(${GLFW_INCLUDE_DIRS})

    # Add TinyObjLoader
    set(TinyObjLoader_DIR ${CMAKE_SOURCE_DIR}/libraries/tinyobjloader) # Adjust this path if necessary
    include_directories(${TinyObjLoader_DIR})
endif()

# source files
include_directories(Vulkan) # include Vulkan classes

add_executable(PathTracer_CPP src/main.cpp #Vulkan all source and header files
        src/Vector3.h
        src/Vector3.cpp
        src/Sphere.h
        src/Sphere.cpp
        src/SceneObject.h
        src/SceneObject.cpp
        src/Ray.h
        src/Ray.cpp
        src/AABCubeCenter.h
        src/AABCubeCenter.cpp
        src/AABCubeBounds.h
        src/AABCubeBounds.cpp
        src/BoundingBox.h
        src/BoundingBox.cpp
        src/BVHNode.h
        src/BVHNode.cpp
        src/Render.h
        src/Render.cpp
        src/Camera.h
        src/Camera.cpp
        src/Config.h
        src/Config.cpp
)

# Link Vulkan and glfw libraries
if (WIN32)
    target_link_libraries(PathTracer_CPP ${Vulkan_LIBRARY} glfw ${tinyObjLoader})

elseif (UNIX)
    target_link_libraries(PathTracer_CPP Vulkan::Vulkan ${GLFW_LIBRARIES} ${tinyObjLoader})

endif()