cmake_minimum_required(VERSION 3.22.1)
project(PathTracer_CPP)

# C++ 23
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# compiler flags
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native")
#set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -march=native")

message(STATUS "Using CXX Flags: ${CMAKE_CXX_FLAGS}")

if (WIN32)  # Windows environment (MinGW)
    # Paths to your SDL2 and SDL2_image development folders
    set(SDL2_DIR ${CMAKE_SOURCE_DIR}/libraries/SDL2-2.30.8)
    set(SDL2_IMAGE_DIR ${CMAKE_SOURCE_DIR}/libraries/SDL2_image-2.8.2)

    # Include SDL2 and SDL2_image headers
    include_directories(${SDL2_DIR}/include)
    include_directories(${SDL2_IMAGE_DIR}/include)

    # Link to SDL2 and SDL2_image libraries (adjust to x64 or x86 as needed)
    link_directories(${SDL2_DIR}/lib/x64)
    link_directories(${SDL2_IMAGE_DIR}/lib/x64)
    set(SDL2_LIBRARIES SDL2main.lib SDL2.lib)
    set(SDL2_IMAGE_LIBRARIES SDL2_image.lib)

elseif (UNIX)   # Linux environment (WSL)
    #find_package(Vulkan REQUIRED)
    #include_directories(${Vulkan_INCLUDE_DIRS})

    # Add SDL2 using system packages
    find_package(SDL2 REQUIRED)
    find_package(SDL2_image REQUIRED)
    include_directories(${SDL2_INCLUDE_DIRS})
    include_directories(${SDL2_IMAGE_INCLUDE_DIRS})
endif()

# Add TinyObjLoader
set(TinyObjLoader_DIR ${CMAKE_SOURCE_DIR}/libraries/tinyobjloader) # Adjust this path if necessary
include_directories(${TinyObjLoader_DIR})
# Add IMGUI
set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/libraries/imgui)
add_library(imgui STATIC
        ${IMGUI_DIR}/imgui.cpp
        ${IMGUI_DIR}/imgui_draw.cpp
        ${IMGUI_DIR}/imgui_tables.cpp
        ${IMGUI_DIR}/imgui_widgets.cpp
        ${IMGUI_DIR}/imgui_impl_sdl2.cpp
        ${IMGUI_DIR}/imgui_impl_sdlrenderer2.cpp
)

# Include directories
target_include_directories(imgui PUBLIC ${IMGUI_DIR})

add_executable(PathTracer_CPP
        src/main.cpp #path tracer source files
        src/Vector2.h
        src/Vector3.h
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
        src/CPUPT.h
        src/CPUPT.cpp
        src/Camera.h
        src/Camera.cpp
        src/Config.h
        src/Config.cpp
        src/MeshObject.h
        src/MeshObject.cpp
        src/LoadMesh.h
        src/LoadMesh.cpp
        src/Triangle.h
        src/Material.h
        src/PathTracer.h
        src/UI.cpp
        src/UI.h
        src/Window.h
        src/Renderer.h
        src/InputManager.h
        src/InputManager.cpp
        src/SystemManager.h
        src/MaterialManager.h
        src/SceneObjectManager.h
        src/BVH.h
        src/BVH.cpp
        src/DirectionSampler.cpp
        src/DirectionSampler.h
        src/SurfaceIntegrator.cpp
        src/SurfaceIntegrator.h
        src/Denoiser.cpp
        src/Denoiser.h
        src/ToneMapper.cpp
        src/ToneMapper.h
        src/Matrix4x4.h
)

# Link Vulkan and libraries
if (WIN32)
    target_link_libraries(PathTracer_CPP
            ${CMAKE_SOURCE_DIR}/libraries/SDL2-2.30.8/lib/x64/SDL2main.lib
            ${CMAKE_SOURCE_DIR}/libraries/SDL2-2.30.8/lib/x64/SDL2.lib
            ${CMAKE_SOURCE_DIR}/libraries/SDL2_image-2.8.2/lib/x64/SDL2_image.lib

            imgui
            ${tinyObjLoader}
    )
elseif (UNIX)
    target_link_libraries(PathTracer_CPP
            ${SDL2_LIBRARIES}  # Use system-installed SDL2 libraries
            imgui
            ${tinyObjLoader}
    )
endif()