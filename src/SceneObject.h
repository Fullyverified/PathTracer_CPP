#ifndef SCENEOBJECT_H
#define SCENEOBJECT_H

#include "Vector3.h"
#include "Ray.h"
#include "Material.h"

class SceneObject {
public:

    [[nodiscard]] virtual std::pair<float, float> getIntersectionDistance(Ray &ray) const = 0;
    [[nodiscard]] virtual Vector3 getPos() const = 0;
    [[nodiscard]] virtual Vector3 getScale() const = 0;
    virtual void getNormal(Ray &ray) const = 0;
    [[nodiscard]] virtual std::pair<Vector3, Vector3> getBounds() = 0;
    [[nodiscard]] virtual Material getMaterial() const = 0;
    virtual void setMaterial(Material material) = 0;
    [[nodiscard]] virtual int getObjID() const = 0;
    virtual void printType() const = 0;
    [[nodiscard]] virtual bool isMesh() const = 0;
    [[nodiscard]] virtual BVHNode* getMeshNode() const = 0;

    virtual ~SceneObject() = default; // deconstructor
private:
protected:
    int objID;
    static int objectCounter;
};


#endif //SCENEOBJECT_H
