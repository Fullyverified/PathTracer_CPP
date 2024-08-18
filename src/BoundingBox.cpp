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
    float invDirX = 1.0f / ray.getDir().getX();
    float invDirY = 1.0f / ray.getDir().getY();
    float invDirZ = 1.0f / ray.getDir().getZ();
    float tmp;

    Vector3 tMin(0, 0, 0), tMax(0, 0, 0);

    if (ray.getDir().getX() == 0) {
        if (ray.getPos().getX() < minBounds.getX() || ray.getPos().getX() > maxBounds.getX()) {
            tMin.setX(std::numeric_limits<float>::infinity()); // No intersection possible on this axis
            tMax.setX(-std::numeric_limits<float>::infinity());
        } else {
            tMin.setX(-std::numeric_limits<float>::infinity()); // No intersection possible on this axis
            tMax.setX(std::numeric_limits<float>::infinity());
        }
    } else {
        tMin.setX((minBounds.getX() - ray.getPos().getX()) * invDirX);
        tMax.setX((maxBounds.getX() - ray.getPos().getX()) * invDirX);
        if (tMin.getX() > tMax.getX()) {
            tmp = tMax.getX();
            tMax.setX(tMin.getX());
            tMin.setX(tmp);
        }
    }

    if (ray.getDir().getY() == 0) {
        if (ray.getPos().getY() < minBounds.getY() || ray.getPos().getY() > maxBounds.getY()) {
            tMin.setY(std::numeric_limits<float>::infinity()); // No intersection possible on this axis
            tMax.setY(-std::numeric_limits<float>::infinity());
        } else {
            tMin.setY(-std::numeric_limits<float>::infinity()); // No intersection possible on this axis
            tMax.setY(std::numeric_limits<float>::infinity());
        }
    } else {
        tMin.setY((minBounds.getY() - ray.getPos().getY()) * invDirY);
        tMax.setY((maxBounds.getY() - ray.getPos().getY()) * invDirY);
        if (tMin.getY() > tMax.getY()) {
            tmp = tMax.getY();
            tMax.setY(tMin.getY());
            tMin.setY(tmp);
        }
    }

    if (ray.getDir().getZ() == 0) {
        if (ray.getPos().getZ() < minBounds.getZ() || ray.getPos().getZ() > maxBounds.getZ()) {
            tMin.setZ(std::numeric_limits<float>::infinity()); // No intersection possible on this axis
            tMax.setZ(-std::numeric_limits<float>::infinity());
        } else {
            tMin.setZ(-std::numeric_limits<float>::infinity()); // No intersection possible on this axis
            tMax.setZ(std::numeric_limits<float>::infinity());
        }
    } else {
        tMin.setZ((minBounds.getZ() - ray.getPos().getZ()) * invDirZ);
        tMax.setZ((maxBounds.getZ() - ray.getPos().getZ()) * invDirZ);
        if (tMin.getZ() > tMax.getZ()) {
            tmp = tMax.getZ();
            tMax.setZ(tMin.getZ());
            tMin.setZ(tmp);
        }
    }

    float tNear = std::max(tMin.getZ(), std::max(tMin.getX(), tMin.getY()));
    float tFar = std::min(tMax.getZ(), std::min(tMax.getX(), tMax.getY()));
    return tNear <= tFar && tFar >= 0;
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
    float invDirX = 1.0f / ray.getDir().getX();
    float invDirY = 1.0f / ray.getDir().getY();
    float invDirZ = 1.0f / ray.getDir().getZ();
    float tmp;

    Vector3 tMin(0, 0, 0), tMax(0, 0, 0);

    if (ray.getDir().getX() == 0) {
        if (ray.getPos().getX() < minBounds.getX() || ray.getPos().getX() > maxBounds.getX()) {
            tMin.setX(std::numeric_limits<float>::infinity()); // No intersection possible on this axis
            tMax.setX(-std::numeric_limits<float>::infinity());
        } else {
            tMin.setX(-std::numeric_limits<float>::infinity()); // No intersection possible on this axis
            tMax.setX(std::numeric_limits<float>::infinity());
        }
    } else {
        tMin.setX((minBounds.getX() - ray.getPos().getX()) * invDirX);
        tMax.setX((maxBounds.getX() - ray.getPos().getX()) * invDirX);
        if (tMin.getX() > tMax.getX()) {
            tmp = tMax.getX();
            tMax.setX(tMin.getX());
            tMin.setX(tmp);
        }
    }

    if (ray.getDir().getY() == 0) {
        if (ray.getPos().getY() < minBounds.getY() || ray.getPos().getY() > maxBounds.getY()) {
            tMin.setY(std::numeric_limits<float>::infinity()); // No intersection possible on this axis
            tMax.setY(-std::numeric_limits<float>::infinity());
        } else {
            tMin.setY(-std::numeric_limits<float>::infinity()); // No intersection possible on this axis
            tMax.setY(std::numeric_limits<float>::infinity());
        }
    } else {
        tMin.setY((minBounds.getY() - ray.getPos().getY()) * invDirY);
        tMax.setY((maxBounds.getY() - ray.getPos().getY()) * invDirY);
        if (tMin.getY() > tMax.getY()) {
            tmp = tMax.getY();
            tMax.setY(tMin.getY());
            tMin.setY(tmp);
        }
    }

    if (ray.getDir().getZ() == 0) {
        if (ray.getPos().getZ() < minBounds.getZ() || ray.getPos().getZ() > maxBounds.getZ()) {
            tMin.setZ(std::numeric_limits<float>::infinity()); // No intersection possible on this axis
            tMax.setZ(-std::numeric_limits<float>::infinity());
        } else {
            tMin.setZ(-std::numeric_limits<float>::infinity()); // No intersection possible on this axis
            tMax.setZ(std::numeric_limits<float>::infinity());
        }
    } else {
        tMin.setZ((minBounds.getZ() - ray.getPos().getZ()) * invDirZ);
        tMax.setZ((maxBounds.getZ() - ray.getPos().getZ()) * invDirZ);
        if (tMin.getZ() > tMax.getZ()) {
            tmp = tMax.getZ();
            tMax.setZ(tMin.getZ());
            tMin.setZ(tmp);
        }
    }

    float tNear = std::max(tMin.getZ(), std::max(tMin.getX(), tMin.getY()));
    float tFar = std::min(tMax.getZ(), std::min(tMax.getX(), tMax.getY()));

    //std::cout << "Int Dis tNear: "<<tNear<<"\n";
    //std::cout << "Int Dis tFar: "<<tFar<<"\n";
    // if tNear < 0, return tFar, else return tNear

    //std::cout<<"Pre computed:"<<"\n";
    //std::cout<<"tNear: "<<tNear<<", tFar: "<<tFar<<"\n";
    if (tNear < 0) {
        return {tFar, tNear};
    }
    return {tNear, tFar};
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