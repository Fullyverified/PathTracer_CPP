#include "Sphere.h"
#include "Vector3.h"
#include "Ray.h"

#include <utility>

Sphere::Sphere(Vector3 pos, float radiusx, float radiusy, float radiusz, float R, float G, float B, float RL, float GL, float BL, float roughness, float refrac, float transp) :
pos(pos), radiusx(radiusx), radiusy(radiusy), radiusz(radiusz), R(R), G(G), B(B), RL(RL), GL(GL), BL(BL), roughness(roughness), refrac(refrac), transp(transp) {

}

bool Sphere::objectCulling(Ray &ray) const {
    const Vector3 centerOrigin = ray.getPos().subtractNew(pos);
    //std::cout << "centerOrigin: ";
    const float invXR = 1.0f / (radiusx * radiusx);
    const float invYR = 1.0f / (radiusy * radiusy);
    const float invZR = 1.0f / (radiusz * radiusz);

    //std::cout << "invXR: " << invXR << ", invYR: " << invYR << ", invZR: " << invZR<<std::endl;

    // a should always = 1
    const float a = (ray.getDir().getX() * ray.getDir().getX() * invXR) + (ray.getDir().getY() * ray.getDir().getY() * invYR) + (ray.getDir().getZ() * ray.getDir().getZ() * invZR);
    // b = 2 * (the dot product of the centerOrigin vector by the direction vector)
    const float b = 2 * ((centerOrigin.getX() * ray.getDir().getX() * invXR) + (centerOrigin.getY() * ray.getDir().getY() * invYR) + (centerOrigin.getZ() * ray.getDir().getZ() * invZR));
    // c = the dot product of centerOrigin by itself, - the radius^2 of the sphere
    const float c = (centerOrigin.getX() * centerOrigin.getX() * invXR) + (centerOrigin.getY() * centerOrigin.getY() * invYR) + (centerOrigin.getZ() * centerOrigin.getZ() * invZR) - 1;

    float discriminant = (b * b) - (4 * (a * c));

    //std::cout<<"a: "<<a<<std::endl;
    //std::cout<<"b: "<<b<<std::endl;
    //std::cout<<"c: "<<c<<std::endl;
    //std::cout<<"discriminant: "<<discriminant<<std::endl;

    if (discriminant < 0) {
        return false;
    }

    float sqrtDiscriminant = std::sqrt(discriminant);
    float sqrt1 = (-b - sqrtDiscriminant) / (2 * a);
    float sqrt2 = (-b + sqrtDiscriminant) / (2 * a);

    //std::cout<<"sqrtdisc: "<<sqrtDiscriminant<<std::endl;
    //std::cout<<"sqrt1: "<<sqrt1<<std::endl;
    //std::cout<<"sqrt2: "<<sqrt2<<std::endl;

    if (sqrt1 >= 0 || sqrt2 >= 0) {
        return true;
    }
    return false;
}

bool Sphere::intersectionCheck(Ray &ray) const {
    const float sradius = 1;
    const Vector3 centerOrigin = ray.getPos().subtractNew(pos);
    const float scaledX = centerOrigin.getX() / radiusx; // swap to xradius etc for ellipsoid
    const float scaledY = centerOrigin.getY() / radiusy;
    const float scaledZ = centerOrigin.getZ() / radiusz;

    const float distanceToC = std::sqrt(scaledX * scaledX + scaledY * scaledY + scaledZ * scaledZ);
    return distanceToC <= sradius;
}

void Sphere::getNormal(Ray &ray) const {
    if (pos.getX() == 0 && pos.getY() == 0 && pos.getZ() == 0) {
        ray.getNormal().set(-ray.getPos().getX() / (radiusx * radiusx),
                            -ray.getPos().getY() / (radiusy * radiusy),
                            -ray.getPos().getZ() / (radiusz * radiusz));
        return;
    }
    ray.getNormal().set(2 * (ray.getPos().getX() - pos.getX()) / (radiusx * radiusx),
                        2 * (ray.getPos().getY() - pos.getY()) / (radiusy * radiusy),
                        2 * (ray.getPos().getZ() - pos.getZ()) / (radiusz * radiusz));
    ray.getNormal().normalise();
}

std::pair<Vector3, Vector3> Sphere::getBounds() const {
    Vector3 min(pos.getX() - radiusx, pos.getY() - radiusy, pos.getZ() - radiusz);
    Vector3 max(pos.getX() + radiusx, pos.getY() + radiusy, pos.getZ() + radiusz);
    return std::make_pair(min, max);
}

std::vector<float> Sphere::getIntersectionDistance(Ray &ray) const {
    const Vector3 centerOrigin = ray.getPos().subtractNew(pos);
    const float invXR = 1.0f / (radiusx * radiusx);
    const float invYR = 1.0f / (radiusy * radiusy);
    const float invZR = 1.0f / (radiusz * radiusz);

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


Vector3 Sphere::getPos() const {
    return pos;
}

std::vector<float> Sphere::getCol() const {
    return std::vector<float> {R, G, B};
}

std::vector<float> Sphere::getLum() const {
    return std::vector<float> {RL, GL, BL};
}

float Sphere::getRough() const {
    return roughness;
}

float Sphere::getRefrac() const {
    return refrac;
}

float Sphere::getTransp() const {
    return transp;
}

void Sphere::printType() const {
    std::cout<<"Type: Sphere"<<std::endl;
}