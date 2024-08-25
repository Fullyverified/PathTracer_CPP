#ifndef SDLWINDOW_H
#define SDLWINDOW_H

#include "SDL.h"

class SDLWindow {

public:

    SDLWindow();

    void createWindow(int &W, int &H);

    void createRenderer();

    void initializeTexture(int &W, int &H);

    void presentScreen(uint8_t* pixels, int &W);

    void destroyWindow();

    SDL_Texture* getTextureBuffer() {return this->textureBuffer;}

    void setRelativeMouse();

private:

    SDL_Window *window = nullptr;
    SDL_Renderer *renderer = nullptr;
    SDL_Texture* textureBuffer = nullptr;
};

#endif //SDLWINDOW_H
