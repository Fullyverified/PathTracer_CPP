#ifndef SCENEOBJECT_H
#define SCENEOBJECT_H

#include <Vector3.h>
#include <Ray.h>
#include <string>
#include <vector>

class SceneObject {
public:

    [[nodiscard]] virtual bool objectCulling(Ray& ray) const = 0;
    [[nodiscard]] virtual bool intersectionCheck(Ray& ray) const = 0;
    [[nodiscard]] virtual Vector3 getPos() const = 0;
    virtual void getNormal(Ray &ray) const = 0;
    [[nodiscard]] virtual std::pair<Vector3, Vector3> getBounds() const = 0;
    [[nodiscard]] virtual std::vector<float> getCol() const = 0;
    [[nodiscard]] virtual std::vector<float> getLum() const = 0;
    [[nodiscard]] virtual float getRough() const = 0;
    [[nodiscard]] virtual float getRefrac() const = 0;
    [[nodiscard]] virtual float getTransp() const = 0;
    [[nodiscard]] virtual std::vector<float> getIntersectionDistance(Ray &ray) const = 0;
    virtual void printType() const = 0;

    virtual ~SceneObject() = default; // deconstructor

    /*SceneObject(const SceneObject& other) = delete; // disable copy constructor
    SceneObject& operator=(const SceneObject& other) = delete; // disable copy assignment*/

private:
};


#endif //SCENEOBJECT_H
