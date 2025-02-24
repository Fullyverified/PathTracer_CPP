#define main SDL_main

#include "SDL.h"
#include "vulkan/vulkan.h"

#include <vector>
#include "Vector3.h"
#include "Camera.h"
#include "Sphere.h"
#include "AABCubeBounds.h"
#include "AABCubeCenter.h"
#include "LoadMesh.h"
#include "MeshObject.h"
#include "Material.h"
#include "Render.h"

#include "PathTracer.h"


int main(int argc, char* argv[]) {

    PathTracer pathTracer;

    pathTracer.run();

    return 0;
}
