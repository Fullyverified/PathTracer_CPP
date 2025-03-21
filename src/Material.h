#ifndef MATERIAL_H
#define MATERIAL_H

#include "Vector3.h"

struct Material {
    std::string name;
    Vector3 colour;
    float roughness;
    float metallic;
    float IOR;
    float transmission;
    float emission;
};

#endif //MATERIAL_H
