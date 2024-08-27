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


int main() {

    std::vector<SceneObject*> SceneObjectsList;

    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,-3,0),Vector3(14,1,7),1,1,1,0,0,0,0.75,1,0)); // floor
    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,3,0),Vector3(14,1,7),1,1,1,0,0,0,0.75,1,0)); // roof

    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(8,0,0),Vector3(1,6,7),1,1,1,0,0,0,0.75,1,0)); // back wall

    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,3,3),Vector3(14,12,1),1,1,1,0,0,0,0,1,0)); // left wall
    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,3,-3),Vector3(14,12,1),1,1,1,0,0,0,0,1,0)); // right wall wall

    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(7,0,-1.6),Vector3(2,6,0.5),1,1,1,40,0,0,0.75,1,0)); // right
    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(7,0,0),Vector3(2,6,0.5),1,1,1,0,40,0,0.75,1,0)); // middle
    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(7,0,1.6),Vector3(2,6,0.5),1,1,1,0,0,40,0.75,1,0)); // left




    //SceneObjectsList.emplace_back(new Sphere(Vector3(4,-1.7,1),0.8,0.8,0.8,1,1,1,0,0,0,1,1,0)); // left sphere on floor
    //SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(5,-1.7,1),Vector3(1,1,1),1,1,1,0,0,0,0,1,0)); // left cube on floor
    //SceneObjectsList.emplace_back(new Sphere(Vector3(4,-1.7,-1),0.8,0.8,0.8,1,1,1,0,0,0,0,1,0)); // right sphere on floor

    SceneObjectsList.emplace_back(new Sphere(Vector3(20,10,0),1,1,1,1,1,1,40,40,40,0.75,1,0)); // light outside

    Camera *cam = new Camera(Vector3(-2, 0, 0), Vector3(1, 0, 0));

    Render render(*cam);
    //render.constructBVHST(SceneObjectsList);
    //render.BVHProfiling();
    render.computePixels(SceneObjectsList);

    for (SceneObject *obj: SceneObjectsList) {
        delete obj; // delete sceneObjects from heap
    }
    delete cam; // delete cam

    return 0;
}
