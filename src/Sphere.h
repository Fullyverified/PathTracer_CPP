#ifndef SPHERE_H
#define SPHERE_H

#include "SceneObject.h"
#include "Vector3.h"
#include "Material.h"

class Sphere : public SceneObject {
public:
    Sphere(Vector3 pos, float radiusx, float radiusy, float radiusz, Material &material);
    std::pair<float, float> getIntersectionDistance(Ray &ray) const override;
    [[nodiscard]] Vector3 getPos() const override {return pos;}
    [[nodiscard]] Vector3 getScale() const override {return Vector3(1, 1, 1);}
    void getNormal(Ray &ray) const override;
    [[nodiscard]] std::pair<Vector3, Vector3> getBounds() override;
    [[nodiscard]] Material getMaterial() const override {return material;}
    void setMaterial(Material material) override {this->material = material;}
    void printType() const override;
    [[nodiscard]] int getObjID() const override;
    [[nodiscard]] bool isMesh() const override {return false;}
    [[nodiscard]] BVHNode* getMeshNode() const override {return nullptr;}

    Sphere(const Sphere& other) = delete; // disable copy constructor
    Sphere& operator=(const Sphere& other) = delete; // disable copy assignment

private:
    Vector3 pos;
    Material material;
    float radiusx, radiusy, radiusz;
};

#endif //SPHERE_H
