#ifndef MATERIAL_H
#define MATERIAL_H

#include <SDL_surface.h>
#include "Vector3.h"

struct Material {
    std::string name;
    Vector3 colour;
    float roughness;
    float metallic;
    float IOR;
    float transmission;
    float emission;
    SDL_Surface* textureMap = nullptr;
    SDL_Surface* roughnessMap = nullptr;
    SDL_Surface* metallicMap = nullptr;
    SDL_Surface* emissionMap = nullptr;
};

#endif //MATERIAL_H
