#include "BoundingBox.h"
#include "Vector3.h"
#include "Ray.h"
#include "BVHNode.h"

#include <chrono>
#include <utility>
#include <limits>

BoundingBox::BoundingBox(const Vector3 &minBounds, const Vector3 &maxBounds) : minBounds(minBounds),
                                                                               maxBounds(maxBounds) {
}

BoundingBox::BoundingBox(const BoundingBox &left, const BoundingBox &right) {
    minBounds.setX(std::min(left.minBounds.getX(), right.minBounds.getX()));
    maxBounds.setX(std::max(left.maxBounds.getX(), right.maxBounds.getX()));
    minBounds.setY(std::min(left.minBounds.getY(), right.minBounds.getY()));
    maxBounds.setY(std::max(left.maxBounds.getY(), right.maxBounds.getY()));
    minBounds.setZ(std::min(left.minBounds.getZ(), right.minBounds.getZ()));
    maxBounds.setZ(std::max(left.maxBounds.getZ(), right.maxBounds.getZ()));
}

BoundingBox::~BoundingBox() = default;

bool BoundingBox::intersectionCheck(Ray &ray) const {
    return minBounds.getX() <= ray.getPos().getX() && maxBounds.getX() >= ray.getPos().getX() &&
           minBounds.getY() <= ray.getPos().getY() && maxBounds.getY() >= ray.getPos().getY() &&
           minBounds.getZ() <= ray.getPos().getZ() && maxBounds.getZ() >= ray.getPos().getZ();
}

std::pair<float, float> BoundingBox::getIntersectionDistance(Ray &ray) const {
    // pre calculate inverse
    float invDirX = 1.0f / ray.getDir().getX();
    float invDirY = 1.0f / ray.getDir().getY();
    float invDirZ = 1.0f / ray.getDir().getZ();

    float txmin = (minBounds.getX() - ray.getPos().getX()) * invDirX;
    float txmax = (maxBounds.getX() - ray.getPos().getX()) * invDirX;
    if (txmin > txmax) std::swap(txmin, txmax);

    float tymin = (minBounds.getY() - ray.getPos().getY()) * invDirY;
    float tymax = (maxBounds.getY() - ray.getPos().getY()) * invDirY;
    if (tymin > tymax) std::swap(tymin, tymax);

    if ((txmin > tymax) || (tymin > txmax)) return {-1.0f, -1.0f}; // early exit

    if (tymin > txmin) txmin = tymin;
    if (tymax < txmax) txmax = tymax;

    float tzmin = (minBounds.getZ() - ray.getPos().getZ()) * invDirZ;
    float tzmax = (maxBounds.getZ() - ray.getPos().getZ()) * invDirZ;
    if (tzmin > tzmax) std::swap(tzmin, tzmax);

    if ((txmin > tzmax) || (tzmin > txmax)) return {-1.0f, -1.0f};

    if (tzmin > txmin) txmin = tzmin;
    if (tzmax < txmax) txmax = tzmax;
    return {txmin, txmax};
}

float BoundingBox::getArea() {
    return std::abs((maxBounds.getX() - minBounds.getX()) * (maxBounds.getY() - minBounds.getY()) * (maxBounds.getZ() - minBounds.getZ()));
}

std::pair<Vector3, Vector3> BoundingBox::getBounds() const {
    return std::make_pair(minBounds, maxBounds);
}

void BoundingBox::updateBounds(const BoundingBox &left, const BoundingBox &right) {
    minBounds.setX(std::min(left.minBounds.getX(), right.minBounds.getX()));
    maxBounds.setX(std::max(left.maxBounds.getX(), right.maxBounds.getX()));
    minBounds.setY(std::min(left.minBounds.getY(), right.minBounds.getY()));
    maxBounds.setY(std::max(left.maxBounds.getY(), right.maxBounds.getY()));
    minBounds.setZ(std::min(left.minBounds.getZ(), right.minBounds.getZ()));
    maxBounds.setZ(std::max(left.maxBounds.getZ(), right.maxBounds.getZ()));
}

void BoundingBox::updateBounds(const BoundingBox *left, const BoundingBox *right) {
    minBounds.setX(std::min(left->minBounds.getX(), right->minBounds.getX()));
    maxBounds.setX(std::max(left->maxBounds.getX(), right->maxBounds.getX()));
    minBounds.setY(std::min(left->minBounds.getY(), right->minBounds.getY()));
    maxBounds.setY(std::max(left->maxBounds.getY(), right->maxBounds.getY()));
    minBounds.setZ(std::min(left->minBounds.getZ(), right->minBounds.getZ()));
    maxBounds.setZ(std::max(left->maxBounds.getZ(), right->maxBounds.getZ()));
}
