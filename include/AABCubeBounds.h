#ifndef AABCUBEBOUNDS_H
#define AABCUBEBOUNDS_H

#include "SceneObject.h"
#include "Vector3.h"

class AABCubeBounds : public SceneObject {
public:
    AABCubeBounds(Vector3 pos, Vector3 length, float R, float G, float B, float RL, float GL, float BL, float roughness, float refrac, float transp);
    [[nodiscard]] bool objectCulling(Ray &ray) const override;
    [[nodiscard]] bool intersectionCheck(Ray &ray) const override;
    [[nodiscard]] Vector3 getPos() const override;
    void getNormal(Ray &ray) const override;
    [[nodiscard]] std::pair<Vector3, Vector3> getBounds() const override;
    [[nodiscard]] std::vector<float> getCol() const override;
    [[nodiscard]] std::vector<float> getLum() const override;
    [[nodiscard]] float getRough() const override;
    [[nodiscard]] float getRefrac() const override;
    [[nodiscard]] float getTransp() const override;
    std::vector<float> getIntersectionDistance(Ray &ray) const override;
    void printType() const override;

    AABCubeBounds(const AABCubeBounds& other) = delete; // disable copy constructor
    AABCubeBounds& operator=(const AABCubeBounds& other) = delete; // disable copy assignment

private:
    Vector3 minBounds, maxBounds, pos;
    float roughness, refrac, transp, R, G, B, RL, GL, BL;
};


#endif //AABCUBEBOUNDS_H
