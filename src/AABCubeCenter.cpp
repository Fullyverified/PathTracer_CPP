#include <AABCubeCenter.h>
#include <Vector3.h>
#include <Ray.h>

#include <utility>
#include <limits>

AABCubeCenter::AABCubeCenter(Vector3 pos, Vector3 length, float R, float G, float B, float RL, float GL, float BL,
                             float roughness, float refrac) : pos(pos), length(length), R(R), G(G), B(B), RL(RL),
                                                              GL(GL), BL(BL), roughness(roughness), refrac(refrac),
                                                              minBounds(pos.x - length.x / 2, pos.y - length.y / 2,
                                                                        pos.z - length.z / 2),
                                                              maxBounds(pos.x + length.x / 2, pos.y + length.y / 2,
                                                                        pos.z + length.z / 2) {
}

bool AABCubeCenter::objectCulling(const Ray &ray) const override {
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
    float tNear = std::max(tMin.z, std::max(tMin.x, tMin.y));
    float tFar = std::max(tMax.z, std::max(tMax.x, tMax.y));
    return tNear <= tFar && tFar >= 0;
}

bool AABCubeCenter::intersectionCheck(const Ray &ray) const override {
    return minBounds.x <= ray.pos.x && maxBounds.x >= ray.pos.x && minBounds.y <= ray.pos.y && maxBounds.y >= ray.pos.y
           && minBounds.z <= ray.pos.z && maxBounds.z >= ray.pos.z;
}

void AABCubeCenter::getNormal(Ray ray) const {
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

std::pair<Vector3, Vector3> AABCubeCenter::getBounds(const Ray &ray) const {
    return std::make_pair(minBounds, maxBounds);
}
