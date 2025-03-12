#include "AABCubeCenter.h"
#include "Vector3.h"
#include "Ray.h"

#include <utility>
#include <limits>

AABCubeCenter::AABCubeCenter(Vector3 pos, Vector3 length, Material* material) : pos(pos), length(length), material(material), dir(1, 1, 1) {
    objID = ++objectCounter;
    minBounds = Vector3(pos.getX() - length.getX() / 2, pos.getY() - length.getY() / 2, pos.getZ() - length.getZ() / 2);
    maxBounds = Vector3(pos.getX() + length.getX() / 2, pos.getY() + length.getY() / 2, pos.getZ() + length.getZ() / 2);
}

void AABCubeCenter::updateBounds() {
    minBounds = Vector3(pos.getX() - length.getX() / 2, pos.getY() - length.getY() / 2, pos.getZ() - length.getZ() / 2);
    maxBounds = Vector3(pos.getX() + length.getX() / 2, pos.getY() + length.getY() / 2, pos.getZ() + length.getZ() / 2);
}

void AABCubeCenter::getNormal(Ray &ray) const {
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
    ray.getNormal().normalise();
}

Vector3 AABCubeCenter::getNormal(Vector3 sampledPoint) const {
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

std::pair<float, float> AABCubeCenter::getIntersectionDistance(Ray &ray) const {
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

Vector3 AABCubeCenter::samplePoint(float r1, float r2) const {
    // Determine which face to sample from
    int faceIndex = static_cast<int>(r1 * 6.0f);
    r1 = (r1 * 6.0f) - faceIndex; // Recalculate r1 for face sampling

    // Calculate dimensions of the rectangle
    Vector3 size = maxBounds - minBounds;

    //x: 6, y: -3, z: -10.25
    //x: 8, y: 3, z: -9.75
    // Calculate the point on the selected face
    Vector3 pointOnFace;
    switch (faceIndex) {
        case 0: // Front face
            pointOnFace = Vector3(minBounds.x, minBounds.y + r2 * size.y, minBounds.z + r1 * size.z);
        break;
        case 1: // Back face
            pointOnFace = Vector3(maxBounds.x, minBounds.y + r2 * size.y, minBounds.z + r1 * size.z);
        break;
        case 2: // Left face
            pointOnFace = Vector3(minBounds.x + r2 * size.x, minBounds.y + r2 * size.y, maxBounds.z);
        break;
        case 3: // Right face
            pointOnFace = Vector3(minBounds.x + r2 * size.x, minBounds.y + r2 * size.y, minBounds.z);
        break;
        case 4: // Top face
            pointOnFace = Vector3(minBounds.x + r2 * size.x, maxBounds.y, minBounds.z + r2 * size.z);
            break;
        case 5: // Bottom face
            pointOnFace = Vector3(minBounds.x + r1 * size.x, minBounds.y, minBounds.z + r2 * size.z);
        break;
    }

    return pointOnFace;
}


void AABCubeCenter::computeArea() {
    float length = fabs(maxBounds.x - minBounds.x);
    float width = fabs(maxBounds.z - minBounds.z);
    float height = (maxBounds.y - minBounds.y);
    area = length * width * height;
}

float AABCubeCenter::getArea() const {
    return area;
}

std::pair<Vector3, Vector3> AABCubeCenter::getBounds() {
    return std::make_pair(minBounds, maxBounds);
}

void AABCubeCenter::printType() const {
    std::cout << "Type: AABCubeCenter" << std::endl;
}

int AABCubeCenter::getObjID() const {
    return objID;
}

std::string AABCubeCenter::getType() const {
    return "Cube Center";
}