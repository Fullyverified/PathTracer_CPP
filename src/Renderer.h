#ifndef RENDERER_H
#define RENDERER_H

#include <SDL.h>
#include "Window.h"

class Renderer {
public:
    Renderer(Window& window, int& width, int& height) {
        renderer = SDL_CreateRenderer(window.getSDLWindow(), -1, SDL_RENDERER_ACCELERATED);

        initializeTexture(width, height);
    }

    ~Renderer() {
        SDL_DestroyRenderer(renderer);
    }

    void clearScreen() {
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255); // Black background
        SDL_RenderClear(renderer);
    }

    void presentScreen() {
        SDL_RenderPresent(renderer);
    }

    SDL_Renderer* getSDLRenderer() {
        return renderer;
    }

    void initializeTexture(int &W, int &H) {
        if (textureBuffer) {  // Check if an existing texture exists
            SDL_DestroyTexture(textureBuffer);
            textureBuffer = nullptr; // Ensure it's reset before re-creating
        }
        textureBuffer = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING, W, H);
    }

    void pushRGBBuffer(uint8_t *pixels, int &W) {
        SDL_UpdateTexture(textureBuffer, nullptr, pixels, W * 3);
        SDL_RenderCopy(renderer, textureBuffer, nullptr, nullptr);
    }

private:
    SDL_Renderer* renderer;
    SDL_Texture* textureBuffer;
};


#endif //RENDERER_H