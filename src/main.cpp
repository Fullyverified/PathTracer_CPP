#include <thread>
#include <vector>
#include <iostream>

#include <Vector3.h>
#include <Camera.h>
#include <Sphere.h>
#include <AABCubeBounds.h>
#include <AABCubeCenter.h>

#include "Render.h"


int main() {
    static int RenderResolutionX = 500;
    static int aspectX = 4;
    static int aspectY = 3;
    static int fov = 30;
    static int frameTime = 100; // Milliseconds
    static int raysPerPixel = 25;
    static int bouncesPerRay = 5;
    static bool ASCIIMode = false;
    static double primaryRayStep = 0.01;
    static double secondaryRayStep = 0.01;
    static bool denoise = false;
    static double denoiseWeight = 0.75;
    static double ISO = 35; // up and down keys to modify

    std::vector<SceneObject*> SceneObjectsList;

    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,-3,0),Vector3(14,1,7),1,1,1,0,0,0,0.75,1)); // floor
    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,3,0),Vector3(14,1,7),1,1,1,0,0,0,0.75,1)); // roof

    SceneObjectsList.emplace_back(new AABCubeBounds(Vector3(8,0,0),Vector3(1,6,7),1,1,1,0,0,0,0.5,1)); // back wall

    SceneObjectsList.emplace_back(new AABCubeBounds(Vector3(10,3,3),Vector3(14,12,1),1,1,1,0,0,0,0.75,1)); // left wall
    SceneObjectsList.emplace_back(new AABCubeBounds(Vector3(10,3,-3),Vector3(14,12,1),1,1,1,0,0,0,0.75,1)); // right wall wall

    SceneObjectsList.emplace_back(new Sphere(Vector3(5,2.5,0),1,0.1,1,1,1,1,40,40,40,0.75,1)); // oval at ceiling

    SceneObjectsList.emplace_back(new Sphere(Vector3(5,-1.7,1),0.8,0.8,0.8,1,1,1,1,1,1,1,1));
    SceneObjectsList.emplace_back(new Sphere(Vector3(5,-1.7,-1),0.8,0.8,0.8,1,1,1,1,1,1,0,1));
    Camera *cam = new Camera(1, RenderResolutionX, fov, aspectX, aspectY, Vector3(-2, 0, 0), Vector3(1, 0, 0));

    Render render;
    render.constructBVHST(SceneObjectsList);
    render.BVHProilfing();

    for (SceneObject *obj: SceneObjectsList) {
        delete obj; // delete sceneObjects from heap
    }
    delete cam; // delete cam

    return 0;
}
