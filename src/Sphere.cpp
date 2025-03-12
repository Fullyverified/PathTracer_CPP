#include "Sphere.h"

#include <numbers>

#include "Vector3.h"
#include "Ray.h"

Sphere::Sphere(Vector3 pos, float radiusx, float radiusy, float radiusz, Material* material) :
pos(pos), scale(radiusx, radiusy, radiusz), material(material), dir(1, 1, 1) {
    objID = ++objectCounter;
}

void Sphere::getNormal(Ray &ray) const {
    if (pos.getX() == 0 && pos.getY() == 0 && pos.getZ() == 0) {
        ray.getNormal().set(-ray.getPos().getX() / (scale.x * scale.x),
                            -ray.getPos().getY() / (scale.y * scale.y),
                            -ray.getPos().getZ() / (scale.z * scale.z));
        ray.getNormal().normalise();
        return;
    }
    ray.getNormal().set(2 * (ray.getPos().getX() - pos.getX()) / (scale.x * scale.x),
                        2 * (ray.getPos().getY() - pos.getY()) / (scale.y * scale.y),
                        2 * (ray.getPos().getZ() - pos.getZ()) / (scale.z * scale.z));
    ray.getNormal().normalise();
}

Vector3 Sphere::getNormal(Vector3 sampledPos) const {
    Vector3 normal;
    if (pos.getX() == 0 && pos.getY() == 0 && pos.getZ() == 0) {
        normal.set(-sampledPos.x / (scale.x * scale.x),
                            -sampledPos.y / (scale.y * scale.y),
                            -sampledPos.z / (scale.z * scale.z));
        return normal;
    }
    normal.set(2 * (sampledPos.x - pos.x) / (scale.x * scale.x),
                        2 * (sampledPos.y - pos.y) / (scale.y * scale.y),
                        2 * (sampledPos.z - pos.z) / (scale.z * scale.z));
    normal.normalise();
    return normal;
}

std::pair<Vector3, Vector3> Sphere::getBounds() {
    Vector3 min(pos.getX() - scale.x, pos.getY() - scale.y, pos.getZ() - scale.z);
    Vector3 max(pos.getX() + scale.x, pos.getY() + scale.y, pos.getZ() + scale.z);
    return std::make_pair(min, max);
}

std::pair<float, float> Sphere::getIntersectionDistance(Ray &ray) const {
    const Vector3 centerOrigin = ray.getPos() - pos;
    const float invXR = 1.0f / (scale.x * scale.x);
    const float invYR = 1.0f / (scale.y * scale.y);
    const float invZR = 1.0f / (scale.z * scale.z);

    // a should always = 1
    const float a = (ray.getDir().getX() * ray.getDir().getX() * invXR) + (ray.getDir().getY() * ray.getDir().getY() * invYR) + (ray.getDir().getZ() * ray.getDir().getZ() * invZR);
    // b = 2 * (the dot product of the centerOrigin vector by the direction vector)
    const float b = 2 * ((centerOrigin.getX() * ray.getDir().getX() * invXR) + (centerOrigin.getY() * ray.getDir().getY() * invYR) + (centerOrigin.getZ() * ray.getDir().getZ() * invZR));
    // c = the dot product of centerOrigin by itself, - the radius^2 of the sphere
    const float c = (centerOrigin.getX() * centerOrigin.getX() * invXR) + (centerOrigin.getY() * centerOrigin.getY() * invYR) + (centerOrigin.getZ() * centerOrigin.getZ() * invZR) - 1;

    float discriminant = (b * b) - (4 * (a * c));

    if (discriminant < 0) {
        return {-1,-1};
    }

    float sqrtDiscriminant = std::sqrt(discriminant);
    float sqrt1 = (-b - sqrtDiscriminant) / (2 * a);
    float sqrt2 = (-b + sqrtDiscriminant) / (2 * a);

    if (sqrt1 > sqrt2) {
        float temp = sqrt1;
        sqrt1 = sqrt2;
        sqrt2 = temp;
    }

    return {sqrt1, sqrt2};
}

Vector3 Sphere::samplePoint(float r1, float r2) const {
    // Convert random variables to spherical coordinates
    float theta = 2.0f * std::numbers::pi * r1; // Azimuthal angle [0, 2π]
    float phi = acos(2.0f * r2 - 1.0f); // Polar angle [0, π]

    // Spherical to Cartesian conversion
    float x = sin(phi) * cos(theta);
    float y = sin(phi) * sin(theta);
    float z = cos(phi);

    // Scale by the sphere's scale and position
    Vector3 point = Vector3(x, y, z) * scale + pos;
    return point;
}

float Sphere::getArea() const {
    return area;
}

void Sphere::computeArea() {
    area = 4.0f * std::numbers::pi * (std::pow(scale.x * scale.y, 1.6) + std::pow(scale.x * scale.z, 1.6) + std::pow(scale.y * scale.z, 1.6)) / 3;;
}

void Sphere::printType() const {
    std::cout<<"Type: Sphere"<<std::endl;
}

int Sphere::getObjID() const {
    return objID;
}

std::string Sphere::getType() const {
    return "Sphere";
}
