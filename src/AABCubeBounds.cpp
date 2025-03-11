#include "AABCubeBounds.h"
#include "Vector3.h"
#include "Ray.h"

#include <utility>
#include <limits>

AABCubeBounds::AABCubeBounds(Vector3 minBounds, Vector3 maxBounds, Material* material) :
minBounds(minBounds), maxBounds(maxBounds), material(material) {
    objID = ++objectCounter;
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

Vector3 AABCubeBounds::getNormal(Vector3 sampledPoint) const {
    Vector3 normal;

    const float px = sampledPoint.x;
    const float py = sampledPoint.y;
    const float pz = sampledPoint.z;

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
       normal.set(-1, 0, 0);
    } else if (minDist == xmax) {
        normal.set(1, 0, 0);
    } else if (minDist == ymin) {
        normal.set(0, -1, 0);
    } else if (minDist == ymax) {
        normal.set(0, 1, 0);
    } else if (minDist == zmin) {
        normal.set(0, 0, -1);
    } else if (minDist == zmax) {
        normal.set(0, 0, 1);
    }
    normal.normalise();
    return normal;
}

std::pair<float, float> AABCubeBounds::getIntersectionDistance(Ray &ray) const {
    // recalculating all this is bad but I wanted the methods to be const so thread safe??
    // idk im knew i might change it
    // pre calculate inverse
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

Vector3 AABCubeBounds::samplePoint(float r1, float r2) const {
    // Determine which face to sample from
    int faceIndex = static_cast<int>(r1 * 6.0f);
    r1 = fmod(r1 * 6.0f, 1.0f); // Recalculate r1 for face sampling

    // Calculate dimensions of the rectangle
    Vector3 size = maxBounds - minBounds;

    // Calculate the point on the selected face
    Vector3 pointOnFace;
    switch (faceIndex) {
        case 0: // Front face (z = maxBounds.z)
            pointOnFace = Vector3(minBounds.x + r1 * size.x, minBounds.y + r2 * size.y, maxBounds.z);
        break;
        case 1: // Back face (z = minBounds.z)
            pointOnFace = Vector3(minBounds.x + r1 * size.x, minBounds.y + r2 * size.y, minBounds.z);
        break;
        case 2: // Left face (x = minBounds.x)
            pointOnFace = Vector3(minBounds.x, minBounds.y + r1 * size.y, minBounds.z + r2 * size.z);
        break;
        case 3: // Right face (x = maxBounds.x)
            pointOnFace = Vector3(maxBounds.x, minBounds.y + r1 * size.y, minBounds.z + r2 * size.z);
        break;
        case 4: // Top face (y = maxBounds.y)
            pointOnFace = Vector3(minBounds.x + r1 * size.x, maxBounds.y, minBounds.z + r2 * size.z);
        break;
        case 5: // Bottom face (y = minBounds.y)
            pointOnFace = Vector3(minBounds.x + r1 * size.x, minBounds.y, minBounds.z + r2 * size.z);
        break;
    }

    return pointOnFace;
}

float AABCubeBounds::getArea() const {
    return area;
}

void AABCubeBounds::computeArea() {
    float bottom = maxBounds.x - minBounds.x * maxBounds.z - minBounds.z;
    float left = maxBounds.x - minBounds.x * maxBounds.y - minBounds.y;
    float front = maxBounds.y - minBounds.y * maxBounds.z - minBounds.z;
    area = (bottom * front * left) * 2;
}

std::pair<Vector3, Vector3> AABCubeBounds::getBounds() {
    return std::make_pair(minBounds, maxBounds);
    }

void AABCubeBounds::printType() const {
    std::cout<<"Type: AABCubeBounds"<<std::endl;
}

int AABCubeBounds::getObjID() const {
    return objID;
}

std::string AABCubeBounds::getType() const {
    return "Cube Bounds";
}
