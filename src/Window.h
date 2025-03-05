#ifndef WINDOW_H
#define WINDOW_H

#include <SDL.h>
#include <iostream>

class Window {
public:
    // Constructor
    Window(const char* title, int& width, int& height)
        : window(nullptr), isRunning(false) {

        // Initialize SDL
        if (SDL_Init(SDL_INIT_VIDEO) != 0) {
            std::cerr << "SDL Initialization failed: " << SDL_GetError() << std::endl;
            return;
        }

        // Create the SDL window
        window = SDL_CreateWindow(title, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, SDL_WINDOW_SHOWN);
        if (!window) {
            std::cerr << "Window creation failed: " << SDL_GetError() << std::endl;
            SDL_Quit();
            return;
        }

        isRunning = true; // Everything initialized successfully
    }

    // Destructor
    ~Window() {
        SDL_DestroyWindow(window);
    }

    // Check if the window is running
    bool running() const { return isRunning; }

    SDL_Window* getSDLWindow() {
        return window;
    }

    void setRelativeMouse() {
        if (SDL_GetRelativeMouseMode() == SDL_TRUE) {
            // If currently enabled, disable it
            SDL_SetRelativeMouseMode(SDL_FALSE);
        } else {
            // If currently disabled, enable it
            SDL_SetRelativeMouseMode(SDL_TRUE);
        }
    }

    void setRelativeMouse(bool state) {
        if (state == false) {
            SDL_SetRelativeMouseMode(SDL_FALSE);
        } else {
            SDL_SetRelativeMouseMode(SDL_TRUE);
        }
    }

private:
    SDL_Window* window;
    bool isRunning;
};



#endif //SDLWINDOW_H
