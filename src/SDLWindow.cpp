#include "SDLWindow.h"

#include <SDL.h>
#include <iostream>

SDLWindow::SDLWindow() {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
    }
}

void SDLWindow::createWindow(int &W, int &H) {
    window = SDL_CreateWindow("Path Tracer", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, W, H, SDL_WINDOW_SHOWN | SDL_WINDOW_ALLOW_HIGHDPI);
    if (window == nullptr) {
        std::cerr << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
    }
    SDL_SetRelativeMouseMode(SDL_TRUE);
}

void SDLWindow::createRenderer() {
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (renderer == nullptr) {
        std::cerr << "SDL_CreateRenderer Error: " << SDL_GetError() << std::endl;
        SDL_DestroyWindow(window);
        SDL_Quit();
    }
}

void SDLWindow::initializeTexture(int &W, int &H) {
    textureBuffer = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING, W, H);
    if (textureBuffer == nullptr) {
        std::cerr << "SDL_CreateTexture Error: " << SDL_GetError() << std::endl;
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
    }
}

void SDLWindow::presentScreen(uint8_t* pixels, int &W) {
    SDL_UpdateTexture(textureBuffer, nullptr, pixels, W * 3);
    SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer, textureBuffer, nullptr, nullptr);
    SDL_RenderPresent(renderer);
}

void SDLWindow::destroyWindow() {
    SDL_DestroyTexture(textureBuffer);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

void SDLWindow::setRelativeMouse() {
    if (SDL_GetRelativeMouseMode() == SDL_TRUE) {
        // If currently enabled, disable it
        SDL_SetRelativeMouseMode(SDL_FALSE);
    } else {
        // If currently disabled, enable it
        SDL_SetRelativeMouseMode(SDL_TRUE);
    }
}


