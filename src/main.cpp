#define SDL_MAIN_HANDLED
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


int main() {

    Material white{Vector3(1, 1, 1), 0.75, 0, 1, 0, 0};
    Material red{Vector3(1, 0, 0), 0.75, 0, 1, 0, 0};
    Material green{Vector3(0, 1, 0), 0.75, 0, 1, 0, 0};
    Material light{Vector3(1, 1, 1), 0.75, 0, 1, 0, 40};

    Material metal{Vector3(1, 1, 1), 0.05, 1, 1, 0, 0};
    Material plastic{Vector3(1, 1, 1), 0.05, 0, 1, 0, 0};


    LoadMesh object;
    object.load("../meshes/companionCubeLow.obj");

    std::vector<SceneObject*> SceneObjectsList;
    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,-3,0),Vector3(14,1,7),white)); // floor
    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,3,0),Vector3(14,1,7),white)); // roof

    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(8,0,0),Vector3(1,6,7),white)); // back wall

    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,3,3),Vector3(14,12,1),red)); // left wall
    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,3,-3),Vector3(14,12,1),green)); // right wall

    //SceneObjectsList.emplace_back(new MeshObject(Vector3(5,-2,1),Vector3(0,0,0),Vector3(1,1,1),object,white));
    SceneObjectsList.emplace_back(new Sphere(Vector3(5,-1.7,1),0.8,0.8,0.8,metal)); // left sphere on floor
    SceneObjectsList.emplace_back(new Sphere(Vector3(5,-1.7,-1),0.8,0.8,0.8,plastic)); // right sphere on floor

    SceneObjectsList.emplace_back(new Sphere(Vector3(5,2.5,0),1,0.1,1,light)); // light on ceiling


    Camera *cam = new Camera(Vector3(-2, 0, 0), Vector3(1, 0, 0));


    Render render(*cam);
    render.computePixels(SceneObjectsList);


    for (SceneObject *obj: SceneObjectsList) {
        delete obj; // delete sceneObjects from heap
    }
    delete cam; // delete cam

    return 0;
}
