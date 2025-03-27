#include "MeshObject.h"

#include <algorithm>

#include "BVHNode.h"
#include "Vector3.h"
#include "Ray.h"

MeshObject::MeshObject(Vector3 pos, Vector3 dir, Vector3 scale, LoadMesh* mesh, Material* material) :
pos(pos), dir(dir), scale(scale), loadedMesh(mesh), material(material) {
    objID = ++objectCounter;

    triangles = loadedMesh->getTriangles();
    bounds = loadedMesh->getBounds();

    //updateMatrix();
}

void MeshObject::getNormal(Ray &ray) const {



    Vector3 bCoords = ray.getBCoords();
    const Triangle* triangle = ray.getTriangle();
    float u = bCoords.x;
    float v = bCoords.y;
    float w = bCoords.z;

    if (material->normalMap != nullptr && loadedMesh->isTexCoords()) { // interpolate normal from triangle data
        // read normal from normal map
        Vector2 uv0 = triangle->uv0;
        Vector2 uv1 = triangle->uv1;
        Vector2 uv2 = triangle->uv2;

        Vector2 uvCoord = uv0 * w + uv1 * u + uv2 * v;

        SDL_Surface* normalMap = material->normalMap;
        Vector3 normal;
        int x = static_cast<int>(uvCoord.x * (normalMap->w - 1));
        int y = static_cast<int>((1.0f - uvCoord.y) * (normalMap->h - 1));

        x = std::clamp(x, 0, normalMap->w - 1);
        y = std::clamp(y, 0, normalMap->h - 1);

        Uint32* pixels = (Uint32*)normalMap->pixels;
        Uint32 pixel = pixels[y * normalMap->w + x];

        Uint8 r, g, b;
        SDL_GetRGB(pixel, normalMap->format, &r, &g, &b);
        normal = Vector3((int)r, (int)g, (int)b);
        normal /= 255.0f;
        normal = normal * 2.0f - 1.0f;
        ray.getNormal().set(normal);
        ray.getNormal().normalise();
        return;
    }

    if (loadedMesh->isVertexNormals()) { // interopolate normal from normal data stored in triangle
        ray.getNormal().set((triangle->n0 * w + triangle->n1 * u + triangle->n2 * v));
        ray.getNormal().normalise();
        return;
    }

    // fall back, compute normal from scratch
    Vector3 edge1 = triangle->v1 - triangle->v0;
    Vector3 edge2 = triangle->v2 -triangle->v0;
    Vector3 faceNormal = edge1.cross(edge2);
    faceNormal.normalise();
    ray.getNormal().set(faceNormal);
}

Vector3 MeshObject::getNormal(Vector3 sampledPos) const {
    // not implementable for now

    return {1, 1, 1};
}

Intersection MeshObject::getIntersectionDistance(Ray &ray) const {
    // Save original pos and dir
    Vector3 rayDir = ray.dir;
    Vector3 rayPos = ray.pos;

    Transform transform = {pos, dir, scale};
    //transform.rayToObj(ray); // transform ray to object space
    meshIntersection result = getMeshNode()->searchBVHTreeMesh(ray, this);
    ray.dir = rayDir; // revert transformation
    ray.pos = rayPos;

    return {result.close, result.close, result.triangle, result.bcoords};
}

MeshObject::meshIntersection MeshObject::intersectTriangles(Ray &ray, BVHNode* leafNode) const {
    Vector3 rayDirection = ray.getDir();

    float value = std::numeric_limits<float>::infinity();
    float bestT = value;
    float bestU = value;
    float bestV = value;
    Triangle* bestTriangle{};

    for (Triangle* triangle : leafNode->getTriangles()) {
        // Grab triangle vertices
        Vector3 v0 = triangle->v0;
        Vector3 v1 = triangle->v1;
        Vector3 v2 = triangle->v2;

        // Apply position and scale transformations
        // Apply position and scale transformations
        /*v0 = (v0 + pos) / scale;
        v1 = (v1 + pos) / scale;
        v2 = (v2 + pos) / scale;*/


        Vector3 edge1 = v1 - v0;
        Vector3 edge2 = v2 - v0;

        Vector3 h = rayDirection.cross(edge2);
        float a = edge1.dot(h);
        if (a > -std::numeric_limits<float>::epsilon() && a < std::numeric_limits<float>::epsilon()) {
            continue; // ray parallel to triangle
        }

        float f = 1.0f / a;
        Vector3 s = ray.getPos() - v0;
        float u = f * s.dot(h);

        if (u < 0.0 || u > 1.0) {
            continue;
        }

        Vector3 q = s.cross(edge1);
        float v = f * rayDirection.dot(q);

        if (v < 0.0f || u + v > 1.0f) {
            continue;
        }

        float t = f * edge2.dot(q);

        if (t < bestT && t >= 0) {
            bestT = t;
            bestU = u;
            bestV = v;
            bestTriangle = triangle;
        }

    }

    // return best intersection
    if (bestTriangle != nullptr) {
        return {nullptr, bestT, bestTriangle, Vector3(bestU, bestV, 1.0f - bestU - bestV)};
    }

    // no intersection
    return {nullptr, -1.0f, nullptr, {0}};

}

Vector3 MeshObject::samplePoint(float r1, float r2) const {

    int triIndex = static_cast<int>(r1 * triangles.size());

    Triangle* tri = triangles[triIndex];

    float sqrtR = std::sqrt(r2);
    float u = 1.0f - sqrtR;
    float v = sqrtR * (1.0f - r2);
    float w = r2 * sqrtR;

    // Calculate the point using barycentric interpolation
    return tri->v0 * u + tri->v1 * v + tri->v2 * w;
}

float MeshObject::getArea() const {
    return area;
}

void MeshObject::computeArea() {
    float total = 0;
    for (Triangle* triangle : triangles) {
        Vector3 v0 = triangle->v0;
        Vector3 v1 = triangle->v1;
        Vector3 v2 = triangle->v2;

        // Compute the cross product of (v1 - v0) and (v2 - v0)
        Vector3 crossProduct = (v1 - v0).cross(v2 - v0);

        // The area is half the magnitude of the cross product
        total += 0.5f * crossProduct.length();
    }
    area = total;
}

void MeshObject::updateMatrix() {
    /*transform = Matrix4x4::translate(pos) *
                      //Matrix4x4::rotateY(rot.y) *
                      Matrix4x4::scale(scale);
    invTransform = transform.inverse();*/
}

std::pair<Vector3, Vector3> MeshObject::getBounds() {
    auto bounds = loadedMesh->getBounds();
    return {bounds.first + pos, bounds.second + pos};
}

int MeshObject::getObjID() const {
    return objID;
}

void MeshObject::printType() const {
    std::cout<<"Type: Mesh"<<std::endl;
}

std::string MeshObject::getType() const {
    return "Mesh";
}