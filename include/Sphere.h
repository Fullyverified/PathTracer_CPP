#ifndef SPHERE_H
#define SPHERE_H
#include "SceneObject.h"
#include "Vector3.h"

class Sphere : public SceneObject{
public:
    Sphere(Vector3 pos, float radius, float roughness, float refrac);

    bool objectCulling(const Ray &ray) const override;

    Vector3 pos;
    float radius, roughness, refrac;

private:
};


#endif //SPHERE_H
