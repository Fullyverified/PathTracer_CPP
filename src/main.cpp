#define SDL_MAIN_HANDLED
#include "SDL.h"
#include "vulkan/vulkan.h"

#include <vector>
#include "Vector3.h"
#include "Camera.h"
#include "Sphere.h"
#include "AABCubeBounds.h"
#include "AABCubeCenter.h"
#include "Render.h"
#include "Config.h"


int main() {

    std::vector<SceneObject*> SceneObjectsList;

    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,-3,0),Vector3(14,1,7),1,1,1,0,0,0,0.75,1,0)); // floor
    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,3,0),Vector3(14,1,7),1,1,1,0,0,0,0.75,1,0)); // roof

    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(8,0,0),Vector3(1,6,7),1,1,1,0,0,0,0.75,1,0)); // back wall

    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,3,3),Vector3(14,12,1),1,0,0,0,0,0,0.75,1,0)); // left wall
    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,3,-3),Vector3(14,12,1),0,1,0,0,0,0,0.75,1,0)); // right wall wall

    //SceneObjectsList.emplace_back(new Sphere(Vector3(5,-1.7,1),0.8,0.8,0.8,1,1,1,0,0,0,1,1.53,1)); // left sphere on floor
    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(5,-1.7,1),Vector3(1,1,1),1,1,1,0,0,0,0,1,0)); // left cube on floor
    SceneObjectsList.emplace_back(new Sphere(Vector3(5,-1.7,-1),0.8,0.8,0.8,1,1,1,0,0,0,0,1,0)); // right sphere on floor

    SceneObjectsList.emplace_back(new Sphere(Vector3(5,2.5,0),1,0.1,1,1,1,1,40,40,40,0.75,1,0)); // light on ceiling

    // Flipped X
    /*SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(-10,3,3),Vector3(14,12,1),1,1,1,0,0,0,0.75,1,0)); // left wall
    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(-10,3,-3),Vector3(14,12,1),1,1,1,0,0,0,0.75,1,0)); // right wall wall

    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(-8,0,0),Vector3(1,6,7),1,1,1,0,0,0,0.75,1,0)); // back wall

    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(-10,-3,0),Vector3(14,1,7),1,1,1,0,0,0,0.75,1,0)); // floor
    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(-10,3,0),Vector3(14,1,7),1,1,1,0,0,0,0.75,1,0)); // roof

    SceneObjectsList.emplace_back(new Sphere(Vector3(-5,-1.7,-1),0.8,0.8,0.8,1,1,1,0,0,0,0,1,0)); // right sphere on floor
    SceneObjectsList.emplace_back(new Sphere(Vector3(-5,-1.7,1),0.8,0.8,0.8,1,1,1,0,0,0,1,1,0)); // left sphere on floor

    SceneObjectsList.emplace_back(new Sphere(Vector3(-5,2.5,0),1,0.1,1,1,1,1,40,40,40,0.75,1,0)); // light on ceiling*/

    // Flipped Z
    /*SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(3,3,-10),Vector3(1,12,14),1,1,1,0,0,0,1,1,0)); // left wall
    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(-3,3,-10),Vector3(1,12,14),1,1,1,0,0,0,1,1,0)); // right wall wall

    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(0,0,-8),Vector3(7,6,1),1,1,1,0,0,0,0.75,1,0)); // back wall

    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(0,-3,-10),Vector3(7,1,14),1,1,1,0,0,0,0.75,1,0)); // floor
    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(0,3,-10),Vector3(7,1,14),1,1,1,0,0,0,0.75,1,0)); // roof

    SceneObjectsList.emplace_back(new Sphere(Vector3(-1,-1.7,-5),0.8,0.8,0.8,1,1,1,0,0,0,0,1,0)); // right sphere on floor
    SceneObjectsList.emplace_back(new Sphere(Vector3(1,-1.7,-5),0.8,0.8,0.8,1,1,1,0,0,0,1,1,0)); // left sphere on floor

    SceneObjectsList.emplace_back(new Sphere(Vector3(0,2.5,-5),1,0.1,1,1,1,1,40,40,40,0.75,1,0)); // light on ceiling*/

    /*SceneObjectsList.emplace_back(new Sphere(Vector3(5,2.5,2.5),1,1,1,1,1,1,0,0,0,1,1,0)); // left top
    SceneObjectsList.emplace_back(new Sphere(Vector3(5,2.5,0),1,1,1,1,1,1,0,0,0,0.8,1,0)); // middle top
    SceneObjectsList.emplace_back(new Sphere(Vector3(5,2.5,-2.5),1,1,1,1,1,1,0,0,0,0.7,1,0)); // left top

    SceneObjectsList.emplace_back(new Sphere(Vector3(5,0,2.5),1,1,1,1,1,1,0,0,0,0.6,1,0)); // middle left
    SceneObjectsList.emplace_back(new Sphere(Vector3(5,0,0),1,1,1,1,1,1,40,40,40,0.5,1,0)); // middle light
    SceneObjectsList.emplace_back(new Sphere(Vector3(5,0,-2.5),1,1,1,1,1,1,0,0,0,0.4,1,0)); // middle right

    SceneObjectsList.emplace_back(new Sphere(Vector3(5,-2.5,2.5),1,1,1,1,1,1,0,0,0,0.3,1,0)); // bottom left
    SceneObjectsList.emplace_back(new Sphere(Vector3(5,-2.5,0),1,1,1,1,1,1,0,0,0,0.2,1,0)); // bottom light
    SceneObjectsList.emplace_back(new Sphere(Vector3(5,-2.5,-2.5),1,1,1,1,1,1,0,0,0,0,1,0)); // bottom right*/

    //Camera *cam = new Camera(config, Vector3(-2, -2.0, -0.4), Vector3(1, 0, 0));
    Camera *cam = new Camera(config, Vector3(-2, 0, 0), Vector3(1, 0, 0));

    Render render(*cam);
    //render.constructBVHST(SceneObjectsList);
    //render.BVHProfiling();
    render.computePixels(SceneObjectsList, *cam);

    for (SceneObject *obj: SceneObjectsList) {
        delete obj; // delete sceneObjects from heap
    }
    delete cam; // delete cam

    return 0;
}
