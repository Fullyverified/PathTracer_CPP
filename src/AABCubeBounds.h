#ifndef AABCUBEBOUNDS_H
#define AABCUBEBOUNDS_H

#include "SceneObject.h"
#include "Vector3.h"
#include "Ray.h"

class AABCubeBounds : public SceneObject {
public:
    AABCubeBounds(Vector3 minBounds, Vector3 maxBounds, float R, float G, float B, float RL, float GL, float BL, float roughness, float refrac, float transp);
    [[nodiscard]] std::pair<float, float> getIntersectionDistance(Ray &ray) const override;
    [[nodiscard]] std::pair<float, float> intersectTriangles(Ray &ray, BVHNode* leafNode) const override {return {0.0f, 0.0f};}
    [[nodiscard]] Vector3 getPos() const override;
    [[nodiscard]] Vector3 getScale() const override {return Vector3(1, 1, 1);}
    void getNormal(Ray &ray) const override;
    [[nodiscard]] std::pair<Vector3, Vector3> getBounds() override;
    [[nodiscard]] Vector3 getCol() const override;
    [[nodiscard]] Vector3 getLum() const override;
    [[nodiscard]] float getRough() const override;
    [[nodiscard]] float getRefrac() const override;
    [[nodiscard]] float getTransp() const override;
    void printType() const override;
    [[nodiscard]] int getObjID() const override;
    [[nodiscard]] bool isMesh() const override {return false;}
    [[nodiscard]] BVHNode* getMeshNode() const override {return nullptr;}

    AABCubeBounds(const AABCubeBounds& other) = delete; // disable copy constructor
    AABCubeBounds& operator=(const AABCubeBounds& other) = delete; // disable copy assignment=

private:
    Vector3 minBounds, maxBounds, pos, colour, luminance;
    float roughness, refrac, transp;
};


#endif //AABCUBEBOUNDS_H
