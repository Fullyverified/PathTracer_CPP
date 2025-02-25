#define main SDL_main

#include "SDL.h"
#include "PathTracer.h"

int main(int argc, char* argv[]) {

    PathTracer pathTracer;
    pathTracer.run();

    return 0;
}
