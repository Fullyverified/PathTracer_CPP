#ifndef RAY_H
#define RAY_H

#include <vector>
#include <array>

#include "Vector3.h"
#include "Triangle.h"

class BVHNode;
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
    void setHitObject(SceneObject *hitObject) { this->sceneObject = hitObject; }
    [[nodiscard]] SceneObject* getHitObject() const { return sceneObject; }

    [[nodiscard]] Vector3& getBCoords() {return bCoords;}

    void setBounceDot(float bounceDot) {this->bounceDot = bounceDot;}
    [[nodiscard]] float getBounceDot() const {return bounceDot;}

    void setTriangle(Triangle* triangle) {this->triangle = triangle;}
    [[nodiscard]] Triangle* getTriangle() const {return triangle;}

    void reset() {
        origin = {0, 0, 0};
        pos = {0, 0, 0};
        dir = {0, 0, 0};
        hitpoint = {0, 0, 0};
        normal = {0, 0, 0};
        bCoords = {0, 0, 0};
        bounceDot = 0;
        hit = false;
        sceneObject = nullptr;
        triangle = nullptr;
    }

private:
    Vector3 origin, pos, dir, hitpoint, normal, bCoords;
    float bounceDot;
    bool hit;
    SceneObject* sceneObject;
    Triangle* triangle;
};


#endif //RAY_H