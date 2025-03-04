#include "AABCubeCenter.h"
#include "Vector3.h"
#include "Ray.h"

#include <utility>
#include <limits>

AABCubeCenter::AABCubeCenter(Vector3 pos, Vector3 length, Material* material)
    : pos(pos), length(length), material(material),
      minBounds(pos.getX() - length.getX() / 2, pos.getY() - length.getY() / 2, pos.getZ() - length.getZ() / 2),
      maxBounds(pos.getX() + length.getX() / 2, pos.getY() + length.getY() / 2, pos.getZ() + length.getZ() / 2) {
    objID = ++objectCounter;
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