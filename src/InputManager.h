#ifndef INPUTMANAGER_H
#define INPUTMANAGER_H

#include <SDL_events.h>

#include "Camera.h"
#include "Window.h"

class InputManager {
public:
    InputManager(Camera *camera, Window *window) : camera(camera), window(window), running(true), lockMouse(true), hideUI(false) {
        running = true;
    }

    ~InputManager() {
    }

    void processInput(SDL_Event &event) {
        if (event.type == SDL_QUIT) {
            running = false;
        }
        if (event.type == SDL_KEYDOWN) {
            if (event.key.keysym.sym == SDLK_DELETE) {
                lockMouse = lockMouse == false;
                window->setRelativeMouse();
            }

            if (event.key.keysym.sym == SDLK_F1) {
                hideUI = hideUI == false;
            }
        }
    }

    void processInputContinuous(Camera *camera, float deltaTime) {

        const Uint8 *inputState = SDL_GetKeyboardState(NULL);
        SDL_GetRelativeMouseState(&mouseX, &mouseY);

        if (inputState[SDL_SCANCODE_UP]) {
            config.increaeISO();
        }
        if (inputState[SDL_SCANCODE_DOWN]) {
            config.decreaseISO();
        }
        if (inputState[SDL_SCANCODE_RIGHT]) {
            config.resetISO();
        }
        if (inputState[SDL_SCANCODE_ESCAPE]) {
            running = false;
        }

        if (inputState[SDL_SCANCODE_W]) {
            camera->moveForward(deltaTime);
        }

        if (inputState[SDL_SCANCODE_S]) {
            camera->moveBackward(deltaTime);
        }

        if (inputState[SDL_SCANCODE_A]) {
            camera->moveLeft(deltaTime);
        }

        if (inputState[SDL_SCANCODE_D]) {
            camera->moveRight(deltaTime);
        }

        if (inputState[SDL_SCANCODE_E]) {
            camera->moveUp(deltaTime);
        }

        if (inputState[SDL_SCANCODE_Q]) {
            camera->moveDown(deltaTime);
        }

        // mouse movement
        if (!lockMouse) {
            if (mouseX != 0 || mouseY != 0) {
                camera->updateDirection(mouseX, mouseY);
            }
        }

    }

    bool getIsRunning() {
        return running;
    }

    bool getHideUI() {
        return hideUI;
    }

private:
    Window *window;
    Camera *camera;
    int mouseX, mouseY;

    bool running, lockMouse, hideUI;
};


#endif //INPUTMANAGER_H
