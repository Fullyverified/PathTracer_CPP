#include "MeshObject.h"
#include "Vector3.h"
#include "Ray.h"

MeshObject::MeshObject(Vector3 pos, Vector3 dir, Vector3 scale, LoadMesh& mesh, float R, float G, float B, float RL, float GL, float BL, float roughness, float refrac, float transp, bool flipY) :
pos(pos), dir(dir), scale(scale), loadedMesh(mesh) ,colour(R, G, B), luminance(RL, GL, BL), roughness(roughness), refrac(refrac), transp(transp), flipY(flipY) {
    objID = ++objectCounter;

    triangles = loadedMesh.getTriangles();
    bounds = loadedMesh.getBounds();
}

void MeshObject::getNormal(Ray &ray) const {

    Vector3 bCoords = ray.getBCoords();
    float u = bCoords.getX();
    float v = bCoords.getY();
    float w = 1.0f - u - v;

    const Triangle* triangle = triangles[ray.getBCoords().getZ()];

    ray.getNormal().set(triangle->n0 * w + triangle->n1 * u + triangle->n2 * v);

    if (flipY) {ray.getNormal().flipY();}

    ray.getNormal().normalise();
}

std::pair<float, float> MeshObject::getIntersectionDistance(Ray &ray) const {
    Vector3 rayDirection = ray.getDir();
    float closest_t = std::numeric_limits<float>::infinity();
    float t, u, v, f;
    int bestIndex;
    for (int i = 0; i < triangles.size(); i++) {
        Vector3 v0 = triangles[i]->v0;
        Vector3 v1 = triangles[i]->v1;
        Vector3 v2 = triangles[i]->v2;

        v0 = v0 * scale + pos;
        v1 = v1 * scale + pos;
        v2 = v2 * scale + pos;

        if (flipY) {
            v0.flipY();
            v1.flipY();
            v2.flipY();
        }

        Vector3 edge1 = v1 - v0;
        Vector3 edge2 = v2 - v0;

        Vector3 h = rayDirection.cross(edge2);
        float a = edge1.dot(h);
        if (a > -std::numeric_limits<float>::epsilon() && a < std::numeric_limits<float>::epsilon()) {
            continue; // ray parallel to triangle
        }

        f = 1.0f / a;
        Vector3 s = ray.getOrigin() - v0;
        float u = f * s.dot(h);

        if (u < 0.0 || u > 1.0) {
            continue;
        }

        Vector3 q = s.cross(edge1);
        v = f * rayDirection.dot(q);

        if (v < 0.0f || u + v > 1.0f) {
            continue;
        }

        t = f * edge2.dot(q);
        if (t > std::numeric_limits<float>::epsilon() && t < closest_t) {
            closest_t = t;
            bestIndex = i;
        }
    }

    if (closest_t == std::numeric_limits<float>::infinity()) {
        return {-1.0f, -1.0f};
    }

    ray.getBCoords().set(u, v, bestIndex);
    return {closest_t, closest_t};
}

std::pair<Vector3, Vector3> MeshObject::getBounds() {
    auto bounds = loadedMesh.getBounds();
    return {bounds.first * scale + pos, bounds.second * scale + pos};
}

Vector3 MeshObject::getPos() const {
    return pos;
}

Vector3 MeshObject::getCol() const {
    return colour;
}

Vector3 MeshObject::getLum() const {
    return luminance;
}

float MeshObject::getRough() const {
    return roughness;
}

float MeshObject::getRefrac() const {
    return refrac;
}

float MeshObject::getTransp() const {
    return transp;
}

int MeshObject::getObjID() const {
    return objID;
}

void MeshObject::printType() const {
    std::cout<<"Type: Mesh"<<std::endl;
}