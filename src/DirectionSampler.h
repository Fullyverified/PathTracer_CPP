#ifndef DIRECTIONSAMPLER_H
#define DIRECTIONSAMPLER_H

#include <random>

#include "Ray.h"
#include "SceneObject.h"
#include "Vector3.h"

class DirectionSampler {
public:
    DirectionSampler() {}
    ~DirectionSampler() {}

    Vector3 SpecularDirection(Ray &ray, const SceneObject &sceneObject, bool flipNormal) const;

    Vector3 DiffuseDirection(Ray &ray, const SceneObject &sceneObject, bool flipNormal) const;

    Vector3 RefractionDirection(Ray &ray, SceneObject &sceneObject) const;

private:
    static thread_local std::mt19937 rng;
};



#endif //DIRECTIONSAMPLER_H
