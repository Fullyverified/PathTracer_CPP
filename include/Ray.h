#ifndef RAY_H
#define RAY_H

#include <vector>
#include <array>

#include "Config.h"
#include "Vector3.h"

class SceneObject;

class Ray {
public:
    Ray(Vector3 origin, Vector3 dir);
    explicit Ray(Vector3 origin);
    Ray();

    void march(const float& distance);
    void updateOrigin(const float& distance);
    [[nodiscard]] Vector3& getPos();
    [[nodiscard]] Vector3& getOrigin();
    [[nodiscard]] Vector3& getDir();
    [[nodiscard]] Vector3& getHitPoint();
    [[nodiscard]] Vector3& getNormal();
    void setHit(bool hit);
    [[nodiscard]] bool getHit() const;
    void setHitObject(SceneObject* sceneObject);
    [[nodiscard]] SceneObject* getHitObject() const;

    void initialize(Ray &primaryRay);

private:
    Vector3 origin, pos, dir, hitpoint, normal;
    bool hit;
    SceneObject* sceneObject;

    Vector3 tMin, tMax; // for storing bounding box information
};


#endif //RAY_H
