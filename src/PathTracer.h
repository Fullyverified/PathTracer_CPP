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
#include "SceneObjectManager.h"

class PathTracer {
public:
    PathTracer() {
    }

    ~PathTracer() {
    }

    void run() {

        Camera *camera = new Camera(Vector3(-3, 0, 0), Vector3(1, 0, 0));
        SystemManager systemManager;

        systemManager.initialize(camera);

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

    }

private:
};

#endif //PATHTRACER_H
