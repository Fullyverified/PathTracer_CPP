#ifndef SCENEOBJECT_H
#define SCENEOBJECT_H

#include <Vector3.h>
#include <Ray.h>
#include <string>

class SceneObject {
public:

    [[nodiscard]] virtual bool objectCulling(const Ray& ray) const = 0;
    [[nodiscard]] virtual bool intersectionCheck(const Ray& ray) const = 0;
    [[nodiscard]] virtual Vector3 getPos() const = 0;
    virtual void getNormal(Ray ray) const = 0;
    [[nodiscard]] virtual std::pair<Vector3, Vector3> getBounds() const = 0;

    virtual ~SceneObject() = default; // deconstructor

    Vector3 pos;
    float R, G, B, RL, GL, BL, roughness, refrac;

    /*SceneObject(const SceneObject& other) = delete; // disable copy constructor
    SceneObject& operator=(const SceneObject& other) = delete; // disable copy assignment*/
};


#endif //SCENEOBJECT_H
