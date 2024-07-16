#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include "Vector3.h"
#include "Ray.h"

class BoundingBox{
public:
    BoundingBox(Vector3 pos, Vector3 length, float R, float G, float B, float RL, float GL, float BL, float roughness, float refrac);

    [nodiscard] bool objectCulling(const Ray &ray) const;
    [nodiscard] bool intersectionCheck(const Ray &ray) const;
    void getNormal(Ray ray) const;
    [nodiscard] std::pair<Vector3, Vector3> getBounds(const Ray& ray) const;
    [nodiscard] float getIntersectionDistance(const Ray &ray) const;
    [nodiscard] float getArea() const;

    Vector3 minBounds, maxBounds;
    float roughness, refrac, R, G, B, RL, GL, BL;

private:

};


#endif //BOUNDINGBOX_H
