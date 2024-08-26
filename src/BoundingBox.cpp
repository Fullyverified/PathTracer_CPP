#include "BoundingBox.h"
#include "Vector3.h"
#include "Ray.h"
#include "BVHNode.h"

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

bool BoundingBox::objectCulling(Ray &ray) const {
    // pre calculate inverse
    // Precompute inverse direction
    float invDirX = 1.0f / ray.getDir().getX();
    float invDirY = 1.0f / ray.getDir().getY();
    float invDirZ = 1.0f / ray.getDir().getZ();

    float tMin, tMax, tyMin, tyMax, tzMin, tzMax;

    // Calculate intersection distances along X axis
    if (invDirX >= 0) {
        tMin = (minBounds.getX() - ray.getPos().getX()) * invDirX;
        tMax = (maxBounds.getX() - ray.getPos().getX()) * invDirX;
    } else {
        tMin = (maxBounds.getX() - ray.getPos().getX()) * invDirX;
        tMax = (minBounds.getX() - ray.getPos().getX()) * invDirX;
    }

    // Calculate intersection distances along Y axis
    if (invDirY >= 0) {
        tyMin = (minBounds.getY() - ray.getPos().getY()) * invDirY;
        tyMax = (maxBounds.getY() - ray.getPos().getY()) * invDirY;
    } else {
        tyMin = (maxBounds.getY() - ray.getPos().getY()) * invDirY;
        tyMax = (minBounds.getY() - ray.getPos().getY()) * invDirY;
    }

    // Early exit if there is no intersection along Y axis
    if ((tMin > tyMax) || (tyMin > tMax)) return {};

    // Update tMin and tMax to the intersection interval along both X and Y axes
    if (tyMin > tMin) tMin = tyMin;
    if (tyMax < tMax) tMax = tyMax;

    // Calculate intersection distances along Z axis
    if (invDirZ >= 0) {
        tzMin = (minBounds.getZ() - ray.getPos().getZ()) * invDirZ;
        tzMax = (maxBounds.getZ() - ray.getPos().getZ()) * invDirZ;
    } else {
        tzMin = (maxBounds.getZ() - ray.getPos().getZ()) * invDirZ;
        tzMax = (minBounds.getZ() - ray.getPos().getZ()) * invDirZ;
    }

    // Early exit if there is no intersection along Z axis
    if ((tMin > tzMax) || (tzMin > tMax)) return {};

    // Update tMin and tMax to the intersection interval along X, Y, and Z axes
    if (tzMin > tMin) tMin = tzMin;
    if (tzMax < tMax) tMax = tzMax;

    // If tMax < 0, the AABB is behind the ray
    if (tMax < 0) return {};

    // If tMin < 0, ray starts inside the box
    if (tMin < 0) tMin = 0;

    return tMin <= tMax && tMax >= 0;
}

bool BoundingBox::intersectionCheck(Ray &ray) const {
    return minBounds.getX() <= ray.getPos().getX() && maxBounds.getX() >= ray.getPos().getX() &&
           minBounds.getY() <= ray.getPos().getY() && maxBounds.getY() >= ray.getPos().getY() &&
           minBounds.getZ() <= ray.getPos().getZ() && maxBounds.getZ() >= ray.getPos().getZ();
}

void BoundingBox::getNormal(Ray &ray) const {
    float epsilon = 0.05;
    float px = ray.getDir().getX();
    float py = ray.getDir().getY();
    float pz = ray.getDir().getZ();

    // x
    if (std::abs(px - minBounds.getX()) < epsilon) {
        ray.getNormal().set(-1, 0, 0);
    } else if (std::abs(px - maxBounds.getX()) < epsilon) {
        ray.getNormal().set(1, 0, 0);
    }
    // y
    else if (std::abs(py - minBounds.getY()) < epsilon) {
        ray.getNormal().set(0, -1, 0);
    } else if (std::abs(py - maxBounds.getY()) < epsilon) {
        ray.getNormal().set(0, 1, 0);
    }
    // z
    else if (std::abs(pz - minBounds.getZ()) < epsilon) {
        ray.getNormal().set(0, 0, -1);
    } else if (std::abs(pz - maxBounds.getZ()) < epsilon) {
        ray.getNormal().set(0, 0, 1);
    }
}

std::vector<float> BoundingBox::getIntersectionDistance(Ray &ray) const {
    // recalculating all this is bad but I wanted the methods to be const so thread safe??
    // idk im knew i might change it
    // pre calculate inverse
    // Precompute inverse direction
    float invDirX = 1.0f / ray.getDir().getX();
    float invDirY = 1.0f / ray.getDir().getY();
    float invDirZ = 1.0f / ray.getDir().getZ();

    float tMin, tMax, tyMin, tyMax, tzMin, tzMax;

    // Calculate intersection distances along X axis
    if (invDirX >= 0) {
        tMin = (minBounds.getX() - ray.getPos().getX()) * invDirX;
        tMax = (maxBounds.getX() - ray.getPos().getX()) * invDirX;
    } else {
        tMin = (maxBounds.getX() - ray.getPos().getX()) * invDirX;
        tMax = (minBounds.getX() - ray.getPos().getX()) * invDirX;
    }

    // Calculate intersection distances along Y axis
    if (invDirY >= 0) {
        tyMin = (minBounds.getY() - ray.getPos().getY()) * invDirY;
        tyMax = (maxBounds.getY() - ray.getPos().getY()) * invDirY;
    } else {
        tyMin = (maxBounds.getY() - ray.getPos().getY()) * invDirY;
        tyMax = (minBounds.getY() - ray.getPos().getY()) * invDirY;
    }

    // Early exit if there is no intersection along Y axis
    if ((tMin > tyMax) || (tyMin > tMax)) return {};

    // Update tMin and tMax to the intersection interval along both X and Y axes
    if (tyMin > tMin) tMin = tyMin;
    if (tyMax < tMax) tMax = tyMax;

    // Calculate intersection distances along Z axis
    if (invDirZ >= 0) {
        tzMin = (minBounds.getZ() - ray.getPos().getZ()) * invDirZ;
        tzMax = (maxBounds.getZ() - ray.getPos().getZ()) * invDirZ;
    } else {
        tzMin = (maxBounds.getZ() - ray.getPos().getZ()) * invDirZ;
        tzMax = (minBounds.getZ() - ray.getPos().getZ()) * invDirZ;
    }

    // Early exit if there is no intersection along Z axis
    if ((tMin > tzMax) || (tzMin > tMax)) return {};

    // Update tMin and tMax to the intersection interval along X, Y, and Z axes
    if (tzMin > tMin) tMin = tzMin;
    if (tzMax < tMax) tMax = tzMax;

    // If tMax < 0, the AABB is behind the ray
    if (tMax < 0) return {};

    // If tMin < 0, ray starts inside the box
    if (tMin < 0) tMin = 0;

    return {tMin, tMax};
}

float BoundingBox::getArea() const {
    return std::abs(
        (maxBounds.getX() - minBounds.getX()) * (maxBounds.getY() - minBounds.getY()) * (maxBounds.getZ() - minBounds.getZ()));
}

std::pair<Vector3, Vector3> BoundingBox::getBounds() const {
    return std::make_pair(minBounds, maxBounds);
}

void BoundingBox::updateBounds(const BoundingBox& left, const BoundingBox& right) {
    minBounds.setX(std::min(left.minBounds.getX(), right.minBounds.getX()));
    maxBounds.setX(std::max(left.maxBounds.getX(), right.maxBounds.getX()));
    minBounds.setY(std::min(left.minBounds.getY(), right.minBounds.getY()));
    maxBounds.setY(std::max(left.maxBounds.getY(), right.maxBounds.getY()));
    minBounds.setZ(std::min(left.minBounds.getZ(), right.minBounds.getZ()));
    maxBounds.setZ(std::max(left.maxBounds.getZ(), right.maxBounds.getZ()));
}