#ifndef PATHTRACER_H
#define PATHTRACER_H

#include "SDL.h"

#include <vector>
#include "Vector3.h"
#include "Camera.h"
#include "Sphere.h"
#include "AABCubeBounds.h"
#include "AABCubeCenter.h"
#include "LoadMesh.h"
#include "MeshObject.h"
#include "Material.h"
#include "SystemManager.h"
#include "MaterialManager.h"

class PathTracer {
public:
    PathTracer() {
    }

    ~PathTracer() {
    }

    void run() {

        Camera *camera = new Camera(Vector3(-3, 0, 0), Vector3(1, 0, 0));
        SystemManager systemManager;
        MaterialManager* materialManager = systemManager.getMaterialManager();

        LoadMesh companionCube;
        companionCube.load("companionCube.obj");

        LoadMesh lucy;
        lucy.load("lucyScaled.obj");

        LoadMesh diamondOBJ;
        diamondOBJ.load("diamond.obj");

        LoadMesh torus;
        torus.load("torus.obj");

        std::cout << "Scene Objects" << std::endl;

        std::vector<SceneObject *> sceneObjectsList;
        //sceneObjectsList.emplace_back(new MeshObject(Vector3(5,-2.5,1),Vector3(1,1,1),Vector3(1,1,1), companionCube, materialManager->getMaterial("White"))); // companion cube

        // BOX1
        sceneObjectsList.emplace_back(new Sphere(Vector3(5, 2.5, 0), 1, 0.1, 1, materialManager->getMaterial("Light"))); // light on ceiling

        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10, -3, 0), Vector3(14, 1, 7), materialManager->getMaterial("White"))); // floor
        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10, 3, 0), Vector3(14, 1, 7), materialManager->getMaterial("White"))); // roof

        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(8, 0, 0), Vector3(1, 6, 7), materialManager->getMaterial("White"))); // back wall

        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10, 3, 3), Vector3(14, 12, 1), materialManager->getMaterial("Red"))); // left wall
        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10, 3, -3), Vector3(14, 12, 1), materialManager->getMaterial("Green"))); // right wall wall

        // Objects of Interest
        sceneObjectsList.emplace_back(new Sphere(Vector3(4.5,-1.7,1.25),0.8,0.8,0.8,materialManager->getMaterial("Metal"))); // left sphere on floor
        sceneObjectsList.emplace_back(new Sphere(Vector3(4.5, -1.7, -1.25), 0.8, 0.8, 0.8, materialManager->getMaterial("Glass"))); // right sphere on floor

        //sceneObjectsList.emplace_back(new MeshObject(Vector3(4.5, -1.5, 1.25), Vector3(1, 1, 1), Vector3(1, 1, 1), torus, materialManager->getMaterial("Copper"))); // statue left

        // BOX 1

        // BOX2
        sceneObjectsList.emplace_back(new Sphere(Vector3(5, 2.5, 10), 1, 0.1, 1, materialManager->getMaterial("Light"))); // light on ceiling

        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10, -3, 10), Vector3(14, 1, 7), materialManager->getMaterial("White"))); // floor
        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10, 3, 10), Vector3(14, 1, 7), materialManager->getMaterial("White"))); // roof

        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(8, 0, 10), Vector3(1, 6, 7), materialManager->getMaterial("White"))); // back wall

        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10, 3, 13), Vector3(14, 12, 1), materialManager->getMaterial("Red"))); // left wall
        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10, 3, 7), Vector3(14, 12, 1), materialManager->getMaterial("Green"))); // right wall wall

        // Objects of Interest
        sceneObjectsList.emplace_back(new MeshObject(Vector3(6, -2.7, 10), Vector3(1, 1, 1), Vector3(1, 1, 1), lucy, materialManager->getMaterial("White"))); // statue left
        sceneObjectsList.emplace_back(new Sphere(Vector3(4.5, -1.7, 8.75), 0.8, 0.8, 0.8, materialManager->getMaterial("Glass"))); // right sphere on floor

        // BOX 2

        // BOX 3

        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,-3,-10),Vector3(14,1,7),materialManager->getMaterial("White"))); // floor
        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,3,-10),Vector3(14,1,7),materialManager->getMaterial("White"))); // roof

        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(8,0,-10),Vector3(1,6,7),materialManager->getMaterial("White"))); // back wall

        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,3,-7),Vector3(14,12,1),materialManager->getMaterial("Mirror"))); // left wall
        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,3,-13),Vector3(14,12,1),materialManager->getMaterial("Mirror"))); // right wall

        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(7,0,-11.6),Vector3(2,6,0.5),materialManager->getMaterial("RedGlow"))); // right
        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(7,0,-10),Vector3(2,6,0.5),materialManager->getMaterial("BlueGlow"))); // middle
        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(7,0,-8.4),Vector3(2,6,0.5),materialManager->getMaterial("GreenGlow"))); // left

        // BOX 3


        systemManager.initialize(sceneObjectsList, camera);

        // start render thread
        systemManager.render();
        std::cout << "Render thread started" << std::endl;

        systemManager.setIsRunning(true);
        SDL_Event event;
        auto lastTime = SDL_GetTicks();

        while (systemManager.getIsRunning()) {
            // Calculate delta time
            auto currentTime = SDL_GetTicks();
            float deltaTime = (currentTime - lastTime) / 1000.0f;
            lastTime = currentTime;

            while (SDL_PollEvent(&event)) {
                // Send inputs to ImGui
                ImGui_ImplSDL2_ProcessEvent(&event);

                if (event.type == SDL_QUIT) {
                    systemManager.setIsRunning(false);
                } else if (event.type == SDL_WINDOWEVENT) {
                    if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
                        /*std::cout << "Window Resized" << std::endl;
                        systemManager.setWindowRes(event.window.data1, event.window.data2);*/
                    }
                } else {
                    systemManager.getInputManager()->processInput(event);
                }
            }
            systemManager.getInputManager()->processInputContinuous(systemManager.getCamera(), deltaTime);

            // Physics
            systemManager.update(deltaTime);

            // Push RGB buffer and present screen
            systemManager.presentScreen();
        }

        // End render thread
        systemManager.renderCleanUp();

        for (SceneObject *obj: sceneObjectsList) {
            delete obj; // delete sceneObjects from heap
        }
    }

private:
};

#endif //PATHTRACER_H
