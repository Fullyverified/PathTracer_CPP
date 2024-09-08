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

    # Add GLM // if your vulkan SDK doesn't include GLM
    #set(GLM_DIR ${CMAKE_SOURCE_DIR}/libraries/glm) # Adjust this path if necessary
    #include_directories(${GLM_DIR})

    # Add SDL
    set(SDL_DIR ${CMAKE_SOURCE_DIR}/libraries/SDL)
    add_subdirectory(${SDL_DIR})
    include_directories(${SDL_DIR}/include)

elseif (UNIX)   # Linux environment (WSL)
    find_package(Vulkan REQUIRED)
    include_directories(${Vulkan_INCLUDE_DIRS})

    # Add GLFW using system packages
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(GLFW REQUIRED glfw3)
    include_directories(${GLFW_INCLUDE_DIRS})

    # Add SDL2 using system packages
    find_package(SDL2 REQUIRED)
    include_directories(${SDL2_INCLUDE_DIRS})
endif()

# Add TinyObjLoader
set(TinyObjLoader_DIR ${CMAKE_SOURCE_DIR}/libraries/tinyobjloader) # Adjust this path if necessary
include_directories(${TinyObjLoader_DIR})

# Add VOLK
set(VOLK_DIR ${CMAKE_SOURCE_DIR}/libraries/volk)
include_directories(${VOLK_DIR})

# Add VMA
set(VMA_DIR ${CMAKE_SOURCE_DIR}/libraries/VMA)
include_directories(${VMA_DIR})

# Add STB_IMAGE
set(STB_IMAGE_DIR ${CMAKE_SOURCE_DIR}/libraries/stb_image)
include_directories(${STB_IMAGE_DIR})

# Add IMGUI
set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/libraries/imgui)
include_directories(${IMGUI_DIR})

# Add FMT
set(FMT_DIR ${CMAKE_SOURCE_DIR}/libraries/fmt)
include_directories(${FMT_DIR})
add_subdirectory(${FMT_DIR})

# Add FASTGLTF
set(FASTGLTF_DIR ${CMAKE_SOURCE_DIR}/libraries/fastgltf)
add_subdirectory(${FASTGLTF_DIR})
include_directories(${FASTGLTF_DIR}/include)

# Add VKBOOTSTRAP
set(VKBOOTSTRAP_DIR ${CMAKE_SOURCE_DIR}/libraries/vkbootstrap)
include_directories(${VKBOOTSTRAP_DIR})

# source files
include_directories(Vulkan) # include Vulkan classes

add_executable(PathTracer_CPP
        src/main.cpp #path tracer source files
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
        # Vulkan and vulkan related dependencies
        Vulkan/vk_main.cpp
        Vulkan/camera.h
        Vulkan/camera.cpp
        Vulkan/meshes.h
        Vulkan/meshes.cpp
        Vulkan/vk_descriptors.h
        Vulkan/vk_descriptors.cpp
        Vulkan/vk_engine.h
        Vulkan/vk_engine.cpp
        Vulkan/vk_images.h
        Vulkan/vk_images.cpp
        Vulkan/vk_initializers.h
        Vulkan/vk_initializers.cpp
        Vulkan/vk_pipelines.cpp
        Vulkan/vk_pipelines.h
        Vulkan/vk_types.h
        src/SDLWindow.h
        src/SDLWindow.cpp
        src/Mesh.h
        src/Mesh.cpp

)

# Link Vulkan and glfw libraries
if (WIN32)
    target_link_libraries(PathTracer_CPP
            ${Vulkan_LIBRARY}
            glfw
            ${tinyObjLoader}
            fmt::fmt
            SDL2::SDL2
            fastgltf::fastgltf

            ${imgui}
            ${vkbootstrap}
            ${volk}
    )
elseif (UNIX)
    target_link_libraries(PathTracer_CPP
            Vulkan::Vulkan
            ${GLFW_LIBRARIES}
            glfw
            ${tinyObjLoader}
            fmt::fmt
            ${SDL2_LIBRARIES}  # Use system-installed SDL2 libraries
            fastgltf

            ${imgui}
            ${vkbootstrap}
            ${volk}
    )
endif()