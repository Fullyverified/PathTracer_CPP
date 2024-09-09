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

    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,3,3),Vector3(14,12,1),1,0,0,0,0,0,0.75,1,0)); // left wall
    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,3,-3),Vector3(14,12,1),0,1,0,0,0,0,0.75,1,0)); // right wall wall

    SceneObjectsList.emplace_back(new Sphere(Vector3(5,-1.5,0),1,1,1,1,0,0,0,0,0,1,1,0)); // big
    SceneObjectsList.emplace_back(new Sphere(Vector3(5,-0.75,-0.85),0.15,0.15,0.15,0,0,1,0,0,0,1,1,0)); // small
    //SceneObjectsList.emplace_back(new Sphere(Vector3(5,-1.5,-3),1,1,1,1,0,0,0,0,0,0,1,0)); // reflection

    SceneObjectsList.emplace_back(new Sphere(Vector3(5,2.5,0),1,0.1,1,1,1,1,40,40,40,0.75,1,0)); // light on ceiling

    Camera *cam = new Camera(Vector3(-2, 0, 0), Vector3(1, 0, 0));

    Render render(*cam);
    render.computePixels(SceneObjectsList);

    for (SceneObject *obj: SceneObjectsList) {
        delete obj; // delete sceneObjects from heap
    }
    delete cam; // delete cam

    return 0;
}
