#ifndef RAY_H
#define RAY_H

#include <vector>
#include "Vector3.h"
#include "Config.h"

class SceneObject;

class Ray {
public:
    Ray(Vector3 origin, Vector3 dir);
    Ray(Vector3 origin);
    Ray();

    void march(const float& distance);
    void updateOrigin(const float& distance);
    [[nodiscard]] Vector3 getPos() const;
    [[nodiscard]] Vector3 getOrigin() const;
    [[nodiscard]] Vector3 getDir() const;
    [[nodiscard]] Vector3 getHitPoint() const;
    [[nodiscard]] Vector3 getNormal() const;
    void setHit(bool hit);
    [[nodiscard]] bool getHit() const;
    void setHitObject(SceneObject* sceneObject);
    [[nodiscard]] SceneObject* getHitObject() const;

    void initialize(Ray &primaryRay);

    void storeHitData(const int &currentBounce, const float &boolHit);

    std::vector<float> sumHitData();
    void clearArray();

private:
    Vector3 origin, pos, dir, hitpoint, normal;
    bool hit;
    SceneObject* sceneObject;
    Config config;

    std::vector<std::vector<float>> luminanceRed, luminanceGreen, luminanceBlue; // for storing hit data as a ray bounces through the scene
};


#endif //RAY_H
