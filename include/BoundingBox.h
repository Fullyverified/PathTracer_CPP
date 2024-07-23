#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include "Vector3.h"
#include "Ray.h"

class BVHNode;

class BoundingBox{
public:
    BoundingBox() = default;
    BoundingBox(const Vector3& minBounds, const Vector3& maxBounds);
    BoundingBox(const BoundingBox& left, const BoundingBox& right);
    ~BoundingBox() = default;

    void updateBounds(const BoundingBox& left, const BoundingBox& right);
    [[nodiscard]] bool objectCulling(const Ray &ray) const;
    [[nodiscard]] bool intersectionCheck(const Ray &ray) const;
    void getNormal(Ray &ray) const;
    [[nodiscard]] std::pair<Vector3, Vector3> getBounds() const;
    [[nodiscard]] float getIntersectionDistance(const Ray &ray) const;
    [[nodiscard]] float getArea() const;

    Vector3 minBounds, maxBounds;
private:

};


#endif //BOUNDINGBOX_H
