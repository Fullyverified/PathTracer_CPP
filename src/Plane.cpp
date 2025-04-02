#include "Plane.h"

#include <numbers>

#include "Vector3.h"
#include "Ray.h"

Plane::Plane(Vector3 pos, Vector3 dir, float length, float width, Material* material) :
pos(pos), dir(dir), length(length), width(width), material(material) {
    objID = ++objectCounter;
}

void Plane::getNormal(Ray &ray) const {

    float cosTheta = Vector3::dot(ray.getDir(), dir);
    Vector3 normal = cosTheta > 0 ? dir * -1.0f : dir;
    ray.getNormal().set(normal);
}

Vector3 Plane::getNormal(Vector3 sampledPos) const {
   return dir;
}

std::pair<Vector3, Vector3> Plane::getBounds() {
    Vector3 up{0, 1, 0};
    Vector3 right{1, 0, 0};
    Vector3 sample = (std::abs(Vector3::dot(dir, up)) > 0.99f) ? right : up;

    Vector3 tangent = Vector3::cross(dir, sample);
    tangent.normalise();
    Vector3 bitangent = Vector3::cross(dir, tangent);
    bitangent.normalise();

    // Half extents
    Vector3 halfWidth = tangent * (width * 0.5f);
    Vector3 halfLength = bitangent * (length * 0.5f);
    Vector3 halfHeight = dir * 0.01; // for BVH

    // Generate 4 corners of the plane
    std::vector<Vector3> corners = {
        pos + halfWidth + halfLength + halfHeight,
        pos + halfWidth + halfLength - halfHeight,
        pos + halfWidth - halfLength + halfHeight,
        pos + halfWidth - halfLength - halfHeight,
        pos - halfWidth + halfLength + halfHeight,
        pos - halfWidth - halfLength + halfHeight,
        pos - halfWidth - halfLength - halfHeight,
        pos - halfWidth + halfLength - halfHeight,
        pos + halfWidth + halfLength - halfHeight
    };

    // Initialize bounds
    Vector3 min = corners[0];
    Vector3 max = corners[0];

    // Expand bounds
    for (const Vector3& corner : corners) {
        min = min.min(corner);  // element-wise min
        max = max.max(corner);  // element-wise max
    }

    return std::make_pair(min, max);
}

Intersection Plane::getIntersectionDistance(Ray &ray) const {

    float cosTheta = Vector3::dot(ray.getDir(), dir);
    if (std::abs(cosTheta) < 1e-6f) {
        return {-1.0f, -1.0f, nullptr};
    }
    float t = Vector3::dot(dir, pos - ray.getPos()) / cosTheta;
    if (t > 0) return {t, t, nullptr};
    return {-1.0f, -1.0f, nullptr};
}

Vector3 Plane::samplePoint(float r1, float r2) const {
    Vector3 sample = dir == Vector3{0, 1, 0} ? Vector3{1, 0, 0} : Vector3{0, 1, 0};

    Vector3 tangent = Vector3::cross(dir, sample);
    tangent.normalise();
    Vector3 bitangent = Vector3::cross(dir, tangent);
    bitangent.normalise();

    //Vector3 point(r1 * tangent, 0, 0);

    return {};
}

float Plane::getArea() const {
    return area;
}

void Plane::computeArea() {
    area = length * width;
    std::cout<<"Area: "<<area<<std::endl;
}

void Plane::printType() const {
    std::cout<<"Type: Plane"<<std::endl;
}

int Plane::getObjID() const {
    return objID;
}

std::string Plane::getType() const {
    return "Plane";
}