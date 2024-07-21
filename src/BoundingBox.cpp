#include <BoundingBox.h>
#include <Vector3.h>
#include <Ray.h>
#include "BVHNode.h"

#include <utility>
#include <limits>

BoundingBox::BoundingBox(const Vector3& minBounds, const Vector3& maxBounds) : minBounds(minBounds), maxBounds(maxBounds) {}
BoundingBox::BoundingBox(const BoundingBox& left, const BoundingBox& right) {
    minBounds.x = std::min(left.minBounds.x, right.minBounds.x);
    maxBounds.x = std::max(left.maxBounds.x, right.maxBounds.x);
    minBounds.y = std::min(left.minBounds.y, right.minBounds.y);
    maxBounds.y = std::max(left.maxBounds.y, right.maxBounds.y);
    minBounds.z = std::min(left.minBounds.z, right.minBounds.z);
    maxBounds.z = std::max(left.maxBounds.z, right.maxBounds.z);
}

bool BoundingBox::objectCulling(const Ray &ray) const {
    // pre calculate inverse
    float invDirX = 1.0f / ray.pos.x;
    float invDirY = 1.0f / ray.pos.y;
    float invDirZ = 1.0f / ray.pos.z;
    float tmp;

    Vector3 tMin(0, 0, 0), tMax(0, 0, 0);

    if (ray.dir.x == 0) {
        if (ray.pos.x < minBounds.x || ray.pos.x > maxBounds.x) {
            tMin.x = std::numeric_limits<float>::infinity(); // No intersection possible on this axis
            tMax.x = -std::numeric_limits<float>::infinity();
        } else {
            tMin.x = -std::numeric_limits<float>::infinity(); // Always intersecting on this axis
            tMax.x = std::numeric_limits<float>::infinity();
        }
    } else {
        tMin.x = (minBounds.x - ray.pos.x) * invDirX;
        tMax.x = (maxBounds.x - ray.pos.x) * invDirX;
        if (tMin.x > tMax.x) {
            tmp = tMax.x;
            tMax.x = tMin.x;
            tMax.x = tmp;
        }
    }

    if (ray.dir.y == 0) {
        if (ray.pos.y < minBounds.y || ray.pos.y > maxBounds.x) {
            tMin.y = std::numeric_limits<float>::infinity(); // No intersection possible on this axis
            tMax.y = -std::numeric_limits<float>::infinity();
        } else {
            tMin.y = -std::numeric_limits<float>::infinity(); // Always intersecting on this axis
            tMax.y = std::numeric_limits<float>::infinity();
        }
    } else {
        tMin.y = (minBounds.y - ray.pos.y) * invDirY;
        tMax.y = (maxBounds.y - ray.pos.y) * invDirY;
        if (tMin.y > tMax.y) {
            tmp = tMax.y;
            tMax.y = tMin.y;
            tMax.y = tmp;
        }
    }

    if (ray.dir.z == 0) {
        if (ray.pos.z < minBounds.z || ray.pos.z > maxBounds.z) {
            tMin.z = std::numeric_limits<float>::infinity(); // No intersection possible on this axis
            tMax.z = -std::numeric_limits<float>::infinity();
        } else {
            tMin.z = -std::numeric_limits<float>::infinity(); // Always intersecting on this axis
            tMax.z = std::numeric_limits<float>::infinity();
        }
    } else {
        tMin.z = (minBounds.z - ray.pos.z) * invDirZ;
        tMax.z = (maxBounds.z - ray.pos.z) * invDirZ;
        if (tMin.z > tMax.z) {
            tmp = tMax.z;
            tMax.z = tMin.z;
            tMax.z = tmp;
        }
    }
    const float tNear = std::max(tMin.z, std::max(tMin.x, tMin.y));
    const float tFar = std::max(tMax.z, std::max(tMax.x, tMax.y));
    return tNear <= tFar && tFar >= 0;
}

bool BoundingBox::intersectionCheck(const Ray &ray) const {
    return minBounds.x <= ray.pos.x && maxBounds.x >= ray.pos.x && minBounds.y <= ray.pos.y && maxBounds.y >= ray.pos.y
           && minBounds.z <= ray.pos.z && maxBounds.z >= ray.pos.z;
}

void BoundingBox::getNormal(Ray ray) const {
    float epsilon = 0.05;
    float px = ray.pos.x;
    float py = ray.pos.y;
    float pz = ray.pos.z;
    // x
    if (std::abs(px - minBounds.x) < epsilon) {
        ray.normal.set(-1, 0, 0);
    } else if (std::abs(px - maxBounds.x) < epsilon) {
        ray.normal.set(1, 0, 0);
    }
    // y
    else if (std::abs(py - minBounds.y) < epsilon) {
        ray.normal.set(0, -1, 0);
    } else if (std::abs(py - maxBounds.y) < epsilon) {
        ray.normal.set(0, 1, 0);
    }
    // z
    else if (std::abs(pz - minBounds.z) < epsilon) {
        ray.normal.set(0, 0, -1);
    } else if (std::abs(pz - maxBounds.z) < epsilon) {
        ray.normal.set(0, 0, 1);
    }
}

float BoundingBox::getIntersectionDistance(const Ray &ray) const {
    // recalculating all this is bad but I wanted the methods to be const so thread safe??
    // idk im knew i might change it
   // pre calculate inverse
    float invDirX = 1.0f / ray.pos.x;
    float invDirY = 1.0f / ray.pos.y;
    float invDirZ = 1.0f / ray.pos.z;
    float tmp;

    Vector3 tMin(0, 0, 0), tMax(0, 0, 0);

    if (ray.dir.x == 0) {
        if (ray.pos.x < minBounds.x || ray.pos.x > maxBounds.x) {
            tMin.x = std::numeric_limits<float>::infinity(); // No intersection possible on this axis
            tMax.x = -std::numeric_limits<float>::infinity();
        } else {
            tMin.x = -std::numeric_limits<float>::infinity(); // Always intersecting on this axis
            tMax.x = std::numeric_limits<float>::infinity();
        }
    } else {
        tMin.x = (minBounds.x - ray.pos.x) * invDirX;
        tMax.x = (maxBounds.x - ray.pos.x) * invDirX;
        if (tMin.x > tMax.x) {
            tmp = tMax.x;
            tMax.x = tMin.x;
            tMax.x = tmp;
        }
    }

    if (ray.dir.y == 0) {
        if (ray.pos.y < minBounds.y || ray.pos.y > maxBounds.x) {
            tMin.y = std::numeric_limits<float>::infinity(); // No intersection possible on this axis
            tMax.y = -std::numeric_limits<float>::infinity();
        } else {
            tMin.y = -std::numeric_limits<float>::infinity(); // Always intersecting on this axis
            tMax.y = std::numeric_limits<float>::infinity();
        }
    } else {
        tMin.y = (minBounds.y - ray.pos.y) * invDirY;
        tMax.y = (maxBounds.y - ray.pos.y) * invDirY;
        if (tMin.y > tMax.y) {
            tmp = tMax.y;
            tMax.y = tMin.y;
            tMax.y = tmp;
        }
    }

    if (ray.dir.z == 0) {
        if (ray.pos.z < minBounds.z || ray.pos.z > maxBounds.z) {
            tMin.z = std::numeric_limits<float>::infinity(); // No intersection possible on this axis
            tMax.z = -std::numeric_limits<float>::infinity();
        } else {
            tMin.z = -std::numeric_limits<float>::infinity(); // Always intersecting on this axis
            tMax.z = std::numeric_limits<float>::infinity();
        }
    } else {
        tMin.z = (minBounds.z - ray.pos.z) * invDirZ;
        tMax.z = (maxBounds.z - ray.pos.z) * invDirZ;
        if (tMin.z > tMax.z) {
            tmp = tMax.z;
            tMax.z = tMin.z;
            tMax.z = tmp;
        }
    }
    const float tNear = std::max(tMin.z, std::max(tMin.x, tMin.y));
    const float tFar = std::max(tMax.z, std::max(tMax.x, tMax.y));

    if (tNear > tFar || tFar > 0) {
        return -1;
    }
    return tNear;
}

float BoundingBox::getArea() const {
    return (maxBounds.x - minBounds.x) * (maxBounds.y - minBounds.y) * (maxBounds.z - minBounds.z);
}


std::pair<Vector3, Vector3> BoundingBox::getBounds() const {
    return std::make_pair(minBounds, maxBounds);
}
