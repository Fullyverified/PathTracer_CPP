#ifndef AABCUBEBOUNDS_H
#define AABCUBEBOUNDS_H

#include "SceneObject.h"
#include "Vector3.h"

class AABCubeBounds : public SceneObject {
public:
    AABCubeBounds(Vector3 pos, Vector3 length, float R, float G, float B, float RL, float GL, float BL, float roughness, float refrac);
    [[nodiscard]] bool objectCulling(const Ray &ray) const override;
    [[nodiscard]] bool intersectionCheck(const Ray &ray) const override;
    [[nodiscard]] Vector3 getPos() const override;
    void getNormal(Ray ray) const override;
    [[nodiscard]] std::pair<Vector3, Vector3> getBounds() const override;

    AABCubeBounds(const AABCubeBounds& other) = delete; // disable copy constructor
    AABCubeBounds& operator=(const AABCubeBounds& other) = delete; // disable copy assignment

    Vector3 minBounds, maxBounds, pos;
    float roughness, refrac, R, G, B, RL, GL, BL;

private:

};


#endif //AABCUBEBOUNDS_H