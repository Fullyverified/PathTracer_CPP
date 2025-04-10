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
    [[nodiscard]] Vector3& getDir() { return dir; }
    [[nodiscard]] Vector3& getNormal() { return normal; }
    void setHitObject(SceneObject *hitObject) { this->sceneObject = hitObject; }
    [[nodiscard]] SceneObject* getHitObject() const { return sceneObject; }

    [[nodiscard]] Vector3& getBCoords() {return bCoords;}

    void setTriangle(Triangle* triangle) {this->triangle = triangle;}
    [[nodiscard]] Triangle* getTriangle() const {return triangle;}

    bool getInternal() {return internal;}
    void setInternal(bool internal) {this->internal = internal;}
    void flipInternal() {internal = internal == false;}

    void setDebugTrue() {debug = true;}
    bool getDebug() {
        return debug;
    }

    void reset() {
        pos = {0, 0, 0};
        dir = {0, 0, 0};
        normal = {0, 0, 0};
        bCoords = {0, 0, 0};
        hit = false;
        sceneObject = nullptr;
        triangle = nullptr;
        internal = false;
    }

    void initialize(Ray other) {
        pos = other.pos;
        dir = other.dir;
        normal = other.normal;
        bCoords = other.bCoords;
        triangle = other.triangle;
        internal = other.internal;
    }

    Vector3 pos, dir, normal, bCoords;
    bool hit, internal, debug;
    SceneObject* sceneObject;
    Triangle* triangle;
private:
};


#endif //RAY_H