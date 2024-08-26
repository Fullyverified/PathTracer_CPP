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

    SceneObjectsList.emplace_back(new AABCubeBounds(Vector3(0,-1,-16),Vector3(32,0,16),1,1,1,0,0,0,0.75,1,0)); // floor
    SceneObjectsList.emplace_back(new AABCubeBounds(Vector3(25,0,-16),Vector3(26,15,16),1,1,1,0,0,0,0.75,1,0)); // back wall

    SceneObjectsList.emplace_back(new AABCubeBounds(Vector3(0,0,-15),Vector3(30,15,-14),1,1,1,0,0,0,0.75,1,0)); // right wall
    SceneObjectsList.emplace_back(new AABCubeBounds(Vector3(0,0,14),Vector3(30,15,15),1,1,1,0,0,0,0.75,1,0)); // left wall

    SceneObjectsList.emplace_back(new Sphere(Vector3(10,15,11),1.5,1.5,1.5,1,1,1,40,40,40,1,1,0)); // light

    // back row
    SceneObjectsList.emplace_back(new Sphere(Vector3(14,2,6),2,2,2,1,1,1,0,0,0,0.9,1,0)); // right
    SceneObjectsList.emplace_back(new Sphere(Vector3(14,2,0),2,2,2,1,1,1,0,0,0,0.3,1,0)); // left
    SceneObjectsList.emplace_back(new Sphere(Vector3(14,2,-6),2,2,2,1,1,1,0,0,0,0.1,1,0)); // middle

    // coloured spheres
    SceneObjectsList.emplace_back(new Sphere(Vector3(9,1,6),1,1,1,1,0,0,0,0,0,0.75,1,0)); // red
    SceneObjectsList.emplace_back(new Sphere(Vector3(9,1,0),1,1,1,0,0,1,0,0,0,0.75,1,0)); // blue
    SceneObjectsList.emplace_back(new Sphere(Vector3(9,1,-6),1,1,1,0,1,0,0,0,0,0.75,1,0)); // green

    // floor rectangles
    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(9,0.125,3),Vector3(3,0.25,3),1,1,1,0,0,0,0.75,1,0)); // floor
    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(9,0.125,-3),Vector3(3,0.25,3),1,1,1,0,0,0,0.75,1,0)); // back wall

    // spheres on rectangles
    SceneObjectsList.emplace_back(new Sphere(Vector3(9,1.25,3),1,1,1,1,1,1,0,0,0,1,1,0)); // Red sphere
    SceneObjectsList.emplace_back(new Sphere(Vector3(9,1.25,-3),1,1,1,1,1,1,0,0,0,0.05,1.53,0.95)); // Glass sphere

    // elevated spheres
    SceneObjectsList.emplace_back(new Sphere(Vector3(15,7.5,8.5),1,1,1,1,1,1,0,0,0,0,1,0)); // left
    SceneObjectsList.emplace_back(new Sphere(Vector3(15,7.5,0),1,1,1,1,1,1,0,0,0,0,1,0)); // middle
    SceneObjectsList.emplace_back(new Sphere(Vector3(15,7.5,-8.5),1,1,1,1,1,1,0,0,0,0,1,0)); // right
	// suggested fOV: 20

    Camera *cam = new Camera(Vector3(-20, 8, 0), Vector3(1, -0.15, 0));

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
