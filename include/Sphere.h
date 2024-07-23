#ifndef SPHERE_H
#define SPHERE_H

#include "SceneObject.h"
#include "Vector3.h"

class Sphere : public SceneObject {
public:
    Sphere(Vector3 pos, float radiusx, float radiusy, float radiusz, float R, float G, float B, float RL, float GL, float BL, float roughness, float refrac);
    [[nodiscard]] bool objectCulling(const Ray &ray) const override;
    [[nodiscard]] bool intersectionCheck(const Ray &ray) const override;
    [[nodiscard]] Vector3 getPos() const override;
    void getNormal(Ray &ray) const override;
    [[nodiscard]] std::pair<Vector3, Vector3> getBounds() const override;
    [[nodiscard]] std::vector<float> getCol() const override;
    [[nodiscard]] std::vector<float> getLum() const override;
    [[nodiscard]] float getRough() const override;
    [[nodiscard]] float getRefrac() const override;

    Sphere(const Sphere& other) = delete; // disable copy constructor
    Sphere& operator=(const Sphere& other) = delete; // disable copy assignment

private:
    Vector3 pos;
    float radiusx, radiusy, radiusz, roughness, refrac, R, G, B, RL, GL, BL;
};

#endif //SPHERE_H
