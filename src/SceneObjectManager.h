#ifndef SCENEOBJECTMANAGER_H
#define SCENEOBJECTMANAGER_H

#include <unordered_map>

#include "SceneObject.h"
#include "LoadMesh.h"

#include "Camera.h"

#include "AABCubeBounds.h"
#include "AABCubeCenter.h"
#include "Sphere.h"
#include "MeshObject.h"

#include "MaterialManager.h"

class SceneObjectManager {
public:
    SceneObjectManager(MaterialManager* materialManager) : materialManager(materialManager) {

        camera = new Camera(Vector3(-3, 0, 0), Vector3(1, 0, 0));

        primativeTypes.emplace_back("Cube");
        primativeTypes.emplace_back("Sphere");

        loadMeshes();
        defaultScene();
    }

    ~SceneObjectManager() {
        for (SceneObject *obj: sceneObjects) {
            delete obj; // delete sceneObjects from heap
        }
    }

    std::vector<const char*>& getPrimativeTypes() {
        return primativeTypes;
    }

    std::vector<const char*>& getMeshTypes() {
        return meshNames;
    }


    std::vector<SceneObject*>& getSceneObjects() {
        return sceneObjects;
    }

    void addPrimative(const char* primative) {

        if (primative == "Cube") {
            std::cout<<"Emplacing Cube"<<std::endl;
            sceneObjects.emplace_back(new AABCubeCenter(Vector3(0, 0, 0), Vector3(1, 1, 1),materialManager->getMaterial("Default")));
        }
        if (primative == "Sphere") {
            sceneObjects.emplace_back(new Sphere(Vector3(0,0,0),1,1,1,materialManager->getMaterial("Default")));
        }

    }

    void addMesh(const char* primative) {

        sceneObjects.emplace_back(new MeshObject(Vector3(0, 0, 0), Vector3(1, 1, 1), Vector3(1, 1, 1), meshTypes[primative], materialManager->getMaterial("Default")));

    }

    void removeSceneObject(SceneObject* sceneObject) {
        // Remove the pointer from the vector
        sceneObjects.erase(
            std::remove(sceneObjects.begin(), sceneObjects.end(), sceneObject),
            sceneObjects.end()
        );
        delete sceneObject;
    }

    void loadMeshes() {

        LoadMesh* sphere = new LoadMesh();
        sphere->load("sphere.obj");
        meshTypes["Sphere"] = sphere;

        LoadMesh* cube = new LoadMesh();
        cube->load("cube.obj");
        meshTypes["Cube"] = cube;

        LoadMesh* torus = new LoadMesh();
        torus->load("torus.obj");
        meshTypes["Torus"] = torus;

        LoadMesh* diamond = new LoadMesh();
        diamond->load("diamond.obj");
        meshTypes["Diamond"] = diamond;

        LoadMesh* diamondFlat = new LoadMesh();
        diamondFlat->load("diamondFlat.obj");
        meshTypes["DiamondFlat"] = diamondFlat;

        LoadMesh* companionCube = new LoadMesh();
        companionCube->load("companionCube.obj");
        meshTypes["Companion Cube"] = companionCube;

        LoadMesh* portalButton = new LoadMesh();
        portalButton->load("portalButton.obj");
        meshTypes["portalButton"] = portalButton;

        LoadMesh* lucy = new LoadMesh();
        lucy->load("lucyScaled.obj");
        meshTypes["Lucy"] = lucy;

        /*LoadMesh* stanfordDragon = new LoadMesh();
        stanfordDragon->load("StanfordDragon.obj");
        meshTypes["StanfordDragon"] = stanfordDragon;*/

        LoadMesh* portalGun = new LoadMesh();
        portalGun->load("portalGun.obj");
        meshTypes["PortalGun"] = portalGun;

        refreshMeshNames();
    }

    void refreshMeshNames() {
        meshNames.clear();

        for (auto &pair : meshTypes) {
            meshNames.emplace_back(pair.first);
        }
    }

    void defaultScene() {

        // BOX1
        sceneObjects.emplace_back(new Sphere(Vector3(5, 2.5, 0), 0.8, 0.1, 0.8, materialManager->getMaterial("Light"))); // light on ceiling

        sceneObjects.emplace_back(new AABCubeCenter(Vector3(10, -3, 0), Vector3(14, 1, 7), materialManager->getMaterial("White"))); // floor
        sceneObjects.emplace_back(new AABCubeCenter(Vector3(10, 3, 0), Vector3(14, 1, 7), materialManager->getMaterial("White"))); // roof

        sceneObjects.emplace_back(new AABCubeCenter(Vector3(8, 0, 0), Vector3(1, 6, 7), materialManager->getMaterial("White"))); // back wall

        sceneObjects.emplace_back(new AABCubeCenter(Vector3(10, 0, 3), Vector3(14, 6, 1), materialManager->getMaterial("Red"))); // left wall
        sceneObjects.emplace_back(new AABCubeCenter(Vector3(10, 0, -3), Vector3(14, 6, 1), materialManager->getMaterial("Green"))); // right wall wall

        // Objects of Interest
        //sceneObjects.emplace_back(new MeshObject(Vector3(4.5, -1.7, 1.25), Vector3(1, 1, 1), Vector3(1, 1, 1), meshTypes["DiamondFlat"], materialManager->getMaterial("Diamond"))); // companion cube
        sceneObjects.emplace_back(new Sphere(Vector3(4.5,-1.7,1.25),0.8,0.8,0.8,materialManager->getMaterial("Metal"))); // left sphere on floor
        sceneObjects.emplace_back(new Sphere(Vector3(4.5, -1.7, -1.25), 0.8, 0.8, 0.8, materialManager->getMaterial("Glass"))); // right sphere on floor

        // BOX 1

        // BOX2
        sceneObjects.emplace_back(new Sphere(Vector3(5, 2.5, 10), 1, 0.1, 1, materialManager->getMaterial("Light"))); // light on ceiling

        sceneObjects.emplace_back(new AABCubeCenter(Vector3(10, -3, 10), Vector3(14, 1, 7), materialManager->getMaterial("White"))); // floor
        sceneObjects.emplace_back(new AABCubeCenter(Vector3(10, 3, 10), Vector3(14, 1, 7), materialManager->getMaterial("White"))); // roof

        sceneObjects.emplace_back(new AABCubeCenter(Vector3(8, 0, 10), Vector3(1, 6, 7), materialManager->getMaterial("White"))); // back wall

        sceneObjects.emplace_back(new AABCubeCenter(Vector3(10, 0, 13), Vector3(14, 6, 1), materialManager->getMaterial("Red"))); // left wall
        sceneObjects.emplace_back(new AABCubeCenter(Vector3(10, 0, 7), Vector3(14, 6, 1), materialManager->getMaterial("Green"))); // right wall wall

        // Objects of Interest
        //sceneObjects.emplace_back(new MeshObject(Vector3(6, -2.7, 10), Vector3(1, 1, 1), Vector3(1, 1, 1), meshTypes["Lucy"], materialManager->getMaterial("White"))); // statue left
        sceneObjects.emplace_back(new MeshObject(Vector3(5, -2.5, 8.75), Vector3(1, 1, 1), Vector3(1, 1, 1), meshTypes["Companion Cube"], materialManager->getMaterial("Weighted Cube"))); // companion cube
        sceneObjects.emplace_back(new MeshObject(Vector3(5, -2.5, 11), Vector3(1, 1, 1), Vector3(1, 1, 1), meshTypes["portalButton"], materialManager->getMaterial("Button"))); // companion cube
        sceneObjects.emplace_back(new MeshObject(Vector3(5, -1.4, 8.45), Vector3(1, 1, 1), Vector3(1, 1, 1), meshTypes["PortalGun"], materialManager->getMaterial("Portal Gun"))); // companion cube

        // BOX 2

        // BOX 3

        sceneObjects.emplace_back(new AABCubeCenter(Vector3(10,-3,-10),Vector3(14,1,7),materialManager->getMaterial("White"))); // floor
        sceneObjects.emplace_back(new AABCubeCenter(Vector3(10,3,-10),Vector3(14,1,7),materialManager->getMaterial("White"))); // roof

        sceneObjects.emplace_back(new AABCubeCenter(Vector3(8,0,-10),Vector3(1,6,7),materialManager->getMaterial("White"))); // back wall

        sceneObjects.emplace_back(new AABCubeCenter(Vector3(10,0,-7),Vector3(14,6,1),materialManager->getMaterial("Mirror"))); // left wall
        sceneObjects.emplace_back(new AABCubeCenter(Vector3(10,0,-13),Vector3(14,6,1),materialManager->getMaterial("Mirror"))); // right wall

        sceneObjects.emplace_back(new AABCubeCenter(Vector3(7,0,-10),Vector3(2,6,0.5),materialManager->getMaterial("GreenGlow"))); // middle
        sceneObjects.emplace_back(new AABCubeCenter(Vector3(7,0,-8.4),Vector3(2,6,0.5),materialManager->getMaterial("BlueGlow"))); // left
        sceneObjects.emplace_back(new AABCubeCenter(Vector3(7,0,-11.6),Vector3(2,6,0.5),materialManager->getMaterial("RedGlow"))); // right

        // BOX 3
    }

    Camera* getCamera() {
        return camera;
    }


    // For Restir DI
    std::vector<SceneObject*>& getEmmisiveObjects() {
        updateEmmisiveObjects();
        return emissiveObjects;
    }

    void updateEmmisiveObjects() {
        emissiveObjects.clear();

        for (SceneObject* sceneObject : sceneObjects) {
            Ray ray;
            if (sceneObject->getMaterial()->emission > 0) {
                emissiveObjects.emplace_back(sceneObject);
            }
        }
    }

private:
    Camera* camera;

    std::vector<const char*> primativeTypes; // Sphere, Cube

    std::vector<const char*> meshNames;
    std::unordered_map<const char*, LoadMesh*> meshTypes;

    std::vector<SceneObject*> sceneObjects; // Master list
    std::vector<SceneObject*> emissiveObjects; // List of emissive objects

    MaterialManager* materialManager;
};

#endif //SCENEOBJECTMANAGER_H
