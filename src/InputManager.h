#ifndef INPUTMANAGER_H
#define INPUTMANAGER_H

#include <SDL_events.h>

#include "Camera.h"
#include "Window.h"
#include "Vector3.h"
#include "UI.h"

class SystemManager;

class InputManager {
public:
    InputManager(Camera *camera, Window *window) : camera(camera), window(window), running(true), lockMouse(true), hideUI(false) {
        running = true;
    }

    ~InputManager() {
    }

    void processInput(SDL_Event &event) {
        mousePos = Vector3(event.button.x, event.button.y, 0);

        if (event.type == SDL_QUIT) {
            running = false;
        }
        if (event.type == SDL_KEYDOWN) {
            /*if (event.key.keysym.sym == SDLK_DELETE) {
                lockMouseOverride = lockMouseOverride == false;
                window->setRelativeMouse(!lockMouseOverride);
            }*/

            if (event.key.keysym.sym == SDLK_F1) {
                hideUI = hideUI == false;
            }
        }

        if (event.type == SDL_MOUSEBUTTONUP) {
            if (event.button.button == SDL_BUTTON_LEFT) {
                if (!UI::isWindowHovered) {
                    getClickedObject(event.button.x, event.button.y);
                    std::cout<<"X: "<<event.button.x<<", Y: "<<event.button.y<<std::endl;
                    //debugRay(event.button.x, event.button.y);
                }
            }
        }

    }

    void processInputContinuous(Camera *camera, float deltaTime) {
        const Uint8 *inputState = SDL_GetKeyboardState(NULL);

        Uint32 mouseButtons = SDL_GetMouseState(&mouseX, &mouseY);
        SDL_GetRelativeMouseState(&mouseX, &mouseY);

        if (mouseButtons & SDL_BUTTON(SDL_BUTTON_LEFT)) {
            // Left mouse button is held down
        }


        if (mouseButtons & SDL_BUTTON(SDL_BUTTON_RIGHT)) {
            // Right mouse button is held down
            window->setRelativeMouse(true);
            lockMouse = false;
        } else {
            window->setRelativeMouse(false);
            lockMouse = true;
        }


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

    void getClickedObject(int x, int y);

    void debugRay(int x, int y);

    void setSystemManager(SystemManager *systemManager);

private:
    Window *window;
    Camera *camera;
    SystemManager *systemManager;
    int mouseX, mouseY;

    bool running, lockMouse, lockMouseOverride, hideUI;

    Vector3 mousePos;
};


#endif //INPUTMANAGER_H
