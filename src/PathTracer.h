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

class PathTracer {
public:
    PathTracer() {
    }

    ~PathTracer() {
    }

    void run() {

        Material white{Vector3(1, 1, 1), 0.75, 0, 1, 0, 0};
        Material red{Vector3(1, 0, 0), 0.75, 0, 1,  0, 0};
        Material green{Vector3(0, 1, 0), 0.75, 0, 1,  0, 0};
        Material blue{Vector3(0, 0, 1), 0.75, 0, 1,  0, 0};
        Material light{Vector3(1, 1, 1), 0.75, 0, 1,  0, 15};

        Material metal{Vector3(1, 1, 1), 0, 1, 1,  0, 0};
        Material copper{Vector3(0.66, 0.5, 0.2), 0, 1, 3,  0, 0};
        Material plastic{Vector3(1, 1, 1), 0, 0, 3,  0, 0};
        Material redPlastic{Vector3(1, 0, 0), 0.8, 0, 1,  0, 0};
        Material greenPlastic{Vector3(0, 1, 0), 0, 0, 1,  0, 0};
        Material bluePlastic{Vector3(0, 0, 1), 0.8, 0, 1,  0, 0};
        Material orangePlastic{Vector3(0.66, 0.5, 0.2), 0, 0, 1,  0, 0};
        Material mirror{Vector3(1, 1, 1), 0.0, 1, 1,  0, 0};
        Material glass{Vector3(1, 1, 1), 0.0, 0, 1.5,  1, 0};


        Material redGlow{Vector3(1, 0, 0), 0.75, 0, 1,  0, 5};
        Material blueGlow{Vector3(0, 1, 0), 0.75, 0, 1,  0, 5};
        Material greenGlow{Vector3(0, 0, 1), 0.75, 0, 1,  0, 5};


        Material smoothPlastic{Vector3(1, 1, 1), 0.05, 0, 1,  0, 0};
        Material smoothMetal{Vector3(1, 1, 1), 0.05, 1, 1,  0, 0};
        Material roughPlastic{Vector3(1, 1, 1), 0.8, 0, 1,  0, 0};
        Material roughMetal{Vector3(1, 1, 1), 0.8, 1, 1,  0, 0};

        LoadMesh companionCube;
        companionCube.load("companionCube.obj");

        LoadMesh lucy;
        lucy.load("lucyScaled.obj");

        std::cout << "Scene Objects" << std::endl;

        std::vector<SceneObject *> sceneObjectsList;
        //sceneObjectsList.emplace_back(new MeshObject(Vector3(5,-2.5,1),Vector3(1,1,1),Vector3(1,1,1), companionCube, white)); // companion cube

        // BOX1
        sceneObjectsList.emplace_back(new Sphere(Vector3(5, 2.5, 0), 1, 0.1, 1, light)); // light on ceiling

        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10, -3, 0), Vector3(14, 1, 7), white)); // floor
        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10, 3, 0), Vector3(14, 1, 7), white)); // roof

        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(8, 0, 0), Vector3(1, 6, 7), white)); // back wall

        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10, 3, 3), Vector3(14, 12, 1), red)); // left wall
        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10, 3, -3), Vector3(14, 12, 1), green)); // right wall wall

        // Spheres
        sceneObjectsList.emplace_back(new Sphere(Vector3(4.5,-1.7,1.25),0.8,0.8,0.8,plastic)); // left sphere on floor
        sceneObjectsList.emplace_back(new Sphere(Vector3(4.5, -1.7, -1.25), 0.8, 0.8, 0.8, glass)); // right sphere on floor

        // BOX 1

        // BOX2
        sceneObjectsList.emplace_back(new Sphere(Vector3(5, 2.5, 10), 1, 0.1, 1, light)); // light on ceiling

        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10, -3, 10), Vector3(14, 1, 7), white)); // floor
        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10, 3, 10), Vector3(14, 1, 7), white)); // roof

        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(8, 0, 10), Vector3(1, 6, 7), white)); // back wall

        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10, 3, 13), Vector3(14, 12, 1), red)); // left wall
        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10, 3, 7), Vector3(14, 12, 1), green)); // right wall wall

        // Spheres
        sceneObjectsList.emplace_back(new MeshObject(Vector3(6, -2.7, 10), Vector3(1, 1, 1), Vector3(1, 1, 1), lucy, white)); // statue left

        sceneObjectsList.emplace_back(new Sphere(Vector3(4.5, -1.25, 8.75), 0.8, 0.8, 0.8, metal)); // right sphere on floor

        // BOX 2

        // BOX 3

        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,-3,-10),Vector3(14,1,7),white)); // floor
        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,3,-10),Vector3(14,1,7),white)); // roof

        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(8,0,-10),Vector3(1,6,7),white)); // back wall

        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,3,-7),Vector3(14,12,1),mirror)); // left wall
        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,3,-13),Vector3(14,12,1),mirror)); // right wall

        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(7,0,-11.6),Vector3(2,6,0.5),redGlow)); // right
        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(7,0,-10),Vector3(2,6,0.5),blueGlow)); // middle
        sceneObjectsList.emplace_back(new AABCubeCenter(Vector3(7,0,-8.4),Vector3(2,6,0.5),greenGlow)); // left

        // BOX 3

        Camera *camera = new Camera(Vector3(-3, 0, 0), Vector3(1, 0, 0));
        std::cout << "Making system manager" << std::endl;
        SystemManager systemManager;
        std::cout << "Initializing system manager" << std::endl;
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
