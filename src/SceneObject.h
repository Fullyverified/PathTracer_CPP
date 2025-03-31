#ifndef SCENEOBJECT_H
#define SCENEOBJECT_H

#include <string>

#include "Vector3.h"
#include "Matrix4x4.h"
#include "Ray.h"
#include "Material.h"

struct Intersection {
    float close = -1.0f;
    float far = -1.0f;
    Triangle* triangle = nullptr;
    Vector3 bCoords = {0};
};

class SceneObject {
public:

    [[nodiscard]] virtual Intersection getIntersectionDistance(Ray &ray) const = 0;
    [[nodiscard]] virtual Vector3 samplePoint (float r1, float r2) const = 0; // Sample random point on an object for Restir GI
    [[nodiscard]] virtual float getArea() const = 0;
    virtual void computeArea() = 0;
    [[nodiscard]] virtual Vector3 getPos() const = 0;
    void virtual setPos(Vector3 newPos) = 0;
    [[nodiscard]] virtual Vector3 getDir() const = 0;
    void virtual setDir(Vector3 newDir) = 0;
    [[nodiscard]] virtual Vector3 getScale() const = 0;
    void virtual setScale(Vector3 newScale) = 0;
    virtual void getNormal(Ray &ray) const = 0;
    [[nodiscard]] virtual Vector3 getNormal(Vector3 sampledPos) const = 0;
    [[nodiscard]] virtual std::pair<Vector3, Vector3> getBounds() = 0;
    [[nodiscard]] virtual Material* getMaterial() const = 0;
    [[nodiscard]] virtual Material* getMaterial(Ray& ray) const = 0;
    virtual void setMaterial(Material* material) = 0;
    [[nodiscard]] virtual int getObjID() const = 0;
    virtual void printType() const = 0;
    [[nodiscard]] virtual std::string getType() const = 0;
    [[nodiscard]] virtual bool isMesh() const = 0;
    [[nodiscard]] virtual BVHNode* getMeshNode() const = 0;

    virtual Matrix4x4 getInvTransform() const = 0;

    virtual ~SceneObject() = default; // deconstructor
private:
protected:
    int objID;
    static int objectCounter;
};


#endif //SCENEOBJECT_H
