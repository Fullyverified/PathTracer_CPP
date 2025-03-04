#ifndef AABCUBEBOUNDS_H
#define AABCUBEBOUNDS_H

#include "SceneObject.h"
#include "Vector3.h"
#include "Ray.h"

class AABCubeBounds : public SceneObject {
public:
    AABCubeBounds(Vector3 minBounds, Vector3 maxBounds, Material &material);
    [[nodiscard]] std::pair<float, float> getIntersectionDistance(Ray &ray) const override;
    [[nodiscard]] Vector3 getPos() const override {return pos;}
    [[nodiscard]] Vector3 getScale() const override {return Vector3(1, 1, 1);}
    [[nodiscard]] Material getMaterial() const override {return material;}
    void setMaterial(Material material) override {this->material = material;}
    void getNormal(Ray &ray) const override;
    [[nodiscard]] std::pair<Vector3, Vector3> getBounds() override;
    void printType() const override;
    [[nodiscard]] int getObjID() const override;
    [[nodiscard]] bool isMesh() const override {return false;}
    [[nodiscard]] BVHNode* getMeshNode() const override {return nullptr;}

    AABCubeBounds(const AABCubeBounds& other) = delete; // disable copy constructor
    AABCubeBounds& operator=(const AABCubeBounds& other) = delete; // disable copy assignment=

private:
    Vector3 minBounds, maxBounds, pos, colour, luminance;
    Material& material;
};


#endif //AABCUBEBOUNDS_H
