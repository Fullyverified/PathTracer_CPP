#include "AABCubeBounds.h"
#include "Vector3.h"
#include "Ray.h"

#include <utility>
#include <limits>

AABCubeBounds::AABCubeBounds(Vector3 minBounds, Vector3 maxBounds, float R, float G, float B, float RL, float GL, float BL, float roughness, float refrac, float transp) : minBounds(minBounds), maxBounds(maxBounds), R(R), G(G),
B(B), RL(RL), GL(GL), BL(BL), roughness(roughness), refrac(refrac), transp(transp) {
    objID = ++objectCounter;
}

bool AABCubeBounds::objectCulling(Ray &ray) const {
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
    const float tNear = std::max(tMin.getX(), std::max(tMin.getY(), tMin.getZ()));
    const float tFar = std::min(tMax.getX(), std::min(tMax.getY(), tMax.getZ()));
    return tNear <= tFar && tFar >= 0;
}

bool AABCubeBounds::intersectionCheck(Ray &ray) const {
    return minBounds.getX() <= ray.getPos().getX() && maxBounds.getX() >= ray.getPos().getX() &&
           minBounds.getY() <= ray.getPos().getY() && maxBounds.getY() >= ray.getPos().getY() &&
           minBounds.getZ() <= ray.getPos().getZ() && maxBounds.getZ() >= ray.getPos().getZ();
}

void AABCubeBounds::getNormal(Ray &ray) const {
    const float px = ray.getPos().getX();
    const float py = ray.getPos().getY();
    const float pz = ray.getPos().getZ();

    // x
    float xmin = std::abs(px - minBounds.getX());
    float xmax = std::abs(px - maxBounds.getX());

    // y
    float ymin = std::abs(py - minBounds.getY());
    float ymax = std::abs(py - maxBounds.getY());

    // z
    float zmin = std::abs(pz - minBounds.getZ());
    float zmax = std::abs(pz - maxBounds.getZ());

    // find smallest value
    float minDist = std::min(xmin, std::min(xmax, std::min(ymin, std::min(ymax, std::min(zmin, zmax)))));

    // set normal according the cloest face of the rays position
    if (minDist == xmin) {
        ray.getNormal().set(-1, 0, 0);
    } else if (minDist == xmax) {
        ray.getNormal().set(1, 0, 0);
    } else if (minDist == ymin) {
        ray.getNormal().set(0, -1, 0);
    } else if (minDist == ymax) {
        ray.getNormal().set(0, 1, 0);
    } else if (minDist == zmin) {
        ray.getNormal().set(0, 0, -1);
    } else if (minDist == zmax) {
        ray.getNormal().set(0, 0, 1);
    }
}

std::vector<float> AABCubeBounds::getIntersectionDistance(Ray &ray) const {
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

    if (tNear < 0 && tFar >= 0) {
        return {0, tFar}; // ray starting inside object
    }
    return {tNear, tFar};
}

std::pair<Vector3, Vector3> AABCubeBounds::getBounds() const {
    return std::make_pair(minBounds, maxBounds);
    }

Vector3 AABCubeBounds::getPos() const {
    return pos;
}

std::vector<float> AABCubeBounds::getCol() const {
    return std::vector<float> {R, G, B};
}

std::vector<float> AABCubeBounds::getLum() const {
    return std::vector<float> {RL, GL, BL};
}

float AABCubeBounds::getRough() const {
    return roughness;
}

float AABCubeBounds::getRefrac() const {
    return refrac;
}

float AABCubeBounds::getTransp() const {
    return transp;
}

void AABCubeBounds::printType() const {
    std::cout<<"Type: AABCubeBounds"<<std::endl;
}

int AABCubeBounds::getObjID() const {
    return objID;
}