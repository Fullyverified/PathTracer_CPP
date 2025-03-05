#include "MeshObject.h"

#include "BVHNode.h"
#include "Vector3.h"
#include "Ray.h"

MeshObject::MeshObject(Vector3 pos, Vector3 dir, Vector3 scale, LoadMesh* mesh, Material* material) :
pos(pos), dir(dir), scale(scale), loadedMesh(mesh), material(material) {
    objID = ++objectCounter;

    triangles = loadedMesh->getTriangles();
    bounds = loadedMesh->getBounds();
}

void MeshObject::getNormal(Ray &ray) const {

    if (ray.getTriangle() == nullptr) {
        ray.getNormal().set(1, 0, 0);
        return;
    }
    Vector3 bCoords = ray.getBCoords();
    const Triangle* triangle = ray.getTriangle();

    float u = bCoords.getX();
    float v = bCoords.getY();
    float w = bCoords.getZ();

    ray.getNormal().set(triangle->n0 * w + triangle->n1 * u + triangle->n2 * v);
    ray.getNormal().normalise();
}

std::pair<float, float> MeshObject::getIntersectionDistance(Ray &ray) const {
    Transform transform = {pos, dir, scale, this};
    transform.rayToObj(ray); // transform ray to object space
    MeshObject::meshIntersection result = getMeshNode()->searchBVHTreeMesh(ray, transform);
    transform.rayToWorld(ray); // revert transformation

    ray.setTriangle(result.triangle);
    ray.getBCoords().set(result.bcoords);

    return {result.close, result.far};
}

MeshObject::meshIntersection MeshObject::intersectTriangles(Ray &ray, BVHNode* leafNode) const {
    Vector3 rayDirection = ray.getDir();

    Triangle* triangle = leafNode->getTriangle();
    Vector3 v0 = triangle->v0;
    Vector3 v1 = triangle->v1;
    Vector3 v2 = triangle->v2;

    v0 = (v0 + pos) / scale;
    v1 = (v1 + pos) / scale;
    v2 = (v2 + pos) / scale;

    Vector3 edge1 = v1 - v0;
    Vector3 edge2 = v2 - v0;

    Vector3 h = rayDirection.cross(edge2);
    float a = edge1.dot(h);
    if (a > -std::numeric_limits<float>::epsilon() && a < std::numeric_limits<float>::epsilon()) {
        return {nullptr, -1.0f, -1.0f, nullptr}; // ray parallel to triangle
    }

    float f = 1.0f / a;
    Vector3 s = ray.getOrigin() - v0;
    float u = f * s.dot(h);

    if (u < 0.0 || u > 1.0) {
        return {nullptr, -1.0f, -1.0f, nullptr};
    }

    Vector3 q = s.cross(edge1);
    float v = f * rayDirection.dot(q);

    if (v < 0.0f || u + v > 1.0f) {
        return {nullptr, -1.0f, -1.0f, nullptr};
    }

    float t = f * edge2.dot(q);

    return {nullptr, t, 0.0f, triangle, (u, v, 1.0f - u - v)};
}


std::pair<Vector3, Vector3> MeshObject::getBounds() {
    auto bounds = loadedMesh->getBounds();
    return {bounds.first * scale + pos, bounds.second * scale + pos};
}

int MeshObject::getObjID() const {
    return objID;
}

void MeshObject::printType() const {
    std::cout<<"Type: Mesh"<<std::endl;
}

std::string MeshObject::getType() const {
    return "Mesh Object";
}
