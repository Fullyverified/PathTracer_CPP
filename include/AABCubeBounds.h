#ifndef AABCUBEBOUNDS_H
#define AABCUBEBOUNDS_H

#include "SceneObject.h"
#include "Vector3.h"

class AABCubeBounds : SceneObject{
public:
    AABCubeBounds(Vector3 pos, Vector3 length, float R, float G, float B, float RL, float GL, float BL, float roughness, float refrac);

    [nodiscard] bool objectCulling(const Ray &ray) const override;
    [nodiscard] bool intersectionCheck(const Ray &ray) const override;
    void getNormal(Ray ray) const override;
    [nodiscard] std::pair<Vector3, Vector3> getBounds(const Ray& ray) const override;

    Vector3 minBounds, maxBounds;
    float roughness, refrac, R, G, B, RL, GL, BL;

private:

};


#endif //AABCUBEBOUNDS_H
