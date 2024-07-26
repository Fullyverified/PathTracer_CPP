#include <thread>
#include <vector>
#include <iostream>

#include <Vector3.h>
#include <Camera.h>
#include <Sphere.h>
#include <AABCubeBounds.h>
#include <AABCubeCenter.h>

#include "Render.h"
#include "Config.h"

int main() {

    Config config;

    std::vector<SceneObject*> SceneObjectsList;

    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,-3,0),Vector3(14,1,7),1,1,1,0,0,0,0.75,1)); // floor
    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,3,0),Vector3(14,1,7),1,1,1,0,0,0,0.75,1)); // roof

    SceneObjectsList.emplace_back(new AABCubeBounds(Vector3(8,0,0),Vector3(1,6,7),1,1,1,0,0,0,0.5,1)); // back wall

    SceneObjectsList.emplace_back(new AABCubeBounds(Vector3(10,3,3),Vector3(14,12,1),1,1,1,0,0,0,0.75,1)); // left wall
    SceneObjectsList.emplace_back(new AABCubeBounds(Vector3(10,3,-3),Vector3(14,12,1),1,1,1,0,0,0,0.75,1)); // right wall wall

    SceneObjectsList.emplace_back(new Sphere(Vector3(5,2.5,0),1,0.1,1,1,1,1,40,40,40,0.75,1)); // oval at ceiling

    SceneObjectsList.emplace_back(new Sphere(Vector3(5,-1.7,1),0.8,0.8,0.8,1,1,1,1,1,1,1,1));
    SceneObjectsList.emplace_back(new Sphere(Vector3(5,-1.7,-1),0.8,0.8,0.8,1,1,1,1,1,1,0,1));

    Camera *cam = new Camera(config, Vector3(-2, 0, 0), Vector3(1, 0, 0));
    Render render(config, *cam);
    //render.constructBVHST(SceneObjectsList);
    //render.BVHProilfing();
    render.computePixels(SceneObjectsList, *cam);

    for (SceneObject *obj: SceneObjectsList) {
        delete obj; // delete sceneObjects from heap
    }
    delete cam; // delete cam
    return 0;
}
