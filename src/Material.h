#ifndef MATERIAL_H
#define MATERIAL_H
#include "Vector3.h"

struct Material {
    Vector3 colour;
    float roughness;
    float metallic;
    float IOR;
    float transmission;
    float emission;
    float microD; // Microfacet Distribution Function (D):
    float geometryMasking; // Geometry (Shadowing-Masking) Term (G):
};

#endif //MATERIAL_H
