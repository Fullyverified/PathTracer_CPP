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
    ~BoundingBox();

    void updateBounds(const BoundingBox& left, const BoundingBox& right);
    void updateBounds(const BoundingBox* left, const BoundingBox* right);
    [[nodiscard]] bool intersectionCheck(Ray &ray) const;
    [[nodiscard]] std::pair<Vector3, Vector3> getBounds() const;
    [[nodiscard]] std::pair<float, float> getIntersectionDistance(Ray &ray) const;
    [[nodiscard]] float getArea();

    Vector3 minBounds, maxBounds;
private:

};


#endif //BOUNDINGBOX_H
