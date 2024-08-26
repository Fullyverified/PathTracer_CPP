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
    [[nodiscard]] Vector3& getPos() { return pos; }
    [[nodiscard]] Vector3& getOrigin() { return origin; }
    [[nodiscard]] Vector3& getDir() { return dir; }
    [[nodiscard]] Vector3& getHitPoint() { return hitpoint; }
    [[nodiscard]] Vector3& getNormal() { return normal; }
    void setHit(bool hit) { this->hit = hit; }
    [[nodiscard]] bool getHit() const { return hit; }
    void setHitObject(SceneObject *hitObject) { this->sceneObject = hitObject; }
    [[nodiscard]] SceneObject* getHitObject() const { return sceneObject; }

    void initialize(Ray &primaryRay);

private:
    Vector3 origin, pos, dir, hitpoint, normal;
    bool hit;
    SceneObject* sceneObject;

    float tNear, tFar; // for storing bounding box information
};


#endif //RAY_H
