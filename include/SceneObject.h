#ifndef SCENEOBJECT_H
#define SCENEOBJECT_H

#include <Vector3.h>
#include <Ray.h>

class SceneObject {

public:

    virtual ~SceneObject() = default;
    [[nodiscard]] virtual bool objectCulling(const Ray& ray) const = 0;
    [[nodiscard]] virtual bool intersectionCheck(const Ray& ray) const = 0;
    [[nodiscard]] virtual Vector3 getBounds(const Ray& ray) const = 0;
    virtual Vector3 getNormal(Ray ray) = 0;


private:

};


#endif //SCENEOBJECT_H
