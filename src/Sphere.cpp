#include <Sphere.h>
#include <Vector3.h>
#include <Ray.h>

#include <utility>

Sphere::Sphere(Vector3 pos, float radiusx, float radiusy, float radiusz, float R, float G, float B, float RL, float GL, float BL, float roughness, float refrac) :
pos(pos), radiusx(radiusx), radiusy(radiusy), radiusz(radiusz), R(R), G(G), B(B), RL(RL), GL(GL), BL(BL), roughness(roughness), refrac(refrac) {

}

bool Sphere::objectCulling(const Ray &ray) const override {
    const Vector3 centerOrigin = ray.pos.subtractNew(pos);
    const float invXR = 1.0f / (radiusx * radiusx);
    const float invYR = 1.0f / (radiusy * radiusy);
    const float invZR = 1.0f / (radiusz * radiusz);

    // a should always = 1
    const float a = (ray.dir.x * ray.dir.x * invXR) + (ray.dir.y * ray.dir.y * invYR) + (ray.dir.z * ray.dir.z * invZR);
    // b = 2 * (the dot product of the centerorigin vector by the direction vector)
    const float intermediary = (centerOrigin.x * ray.dir.x * invXR) + (centerOrigin.y * ray.dir.y * invYR) +(centerOrigin.z * ray.dir.z * invZR);
    const float b = 2 * intermediary;
    // c = the dot product of centerorigin by itself, - the radius^2 of the sphere
    const float c = intermediary - 1;

    float discriminant = (b * b) - (4 * (a * c));

    if (discriminant < 0) {
        return false;
    }

    double sqrtDiscriminant = std::sqrt(discriminant);
    double sqrt1 = (-b - sqrtDiscriminant) / (2 * a);
    double sqrt2 = (-b + sqrtDiscriminant) / (2 * a);

    if (sqrt1 >= 0 || sqrt2 >= 0) {
        return true;
    }

    return false;
}

bool Sphere::intersectionCheck(const Ray &ray) const override {
    const float sradius = 1;
    const Vector3 centerOrigin = ray.pos.subtractNew(pos);
    const float scaledX = centerOrigin.x / radiusx; // swap to xradius etc for ellipsoid
    const float scaledY = centerOrigin.y / radiusy;
    const float scaledZ = centerOrigin.z / radiusz;

    const float distanceToC = std::sqrt(scaledX * scaledX + scaledY * scaledY + scaledZ * scaledZ);
    return distanceToC <= sradius;
}

void Sphere::getNormal(Ray ray) const {
    if (pos.x == 0 && pos.y == 0 && pos.z == 0) {
        ray.normal.x = -ray.pos.x / (radiusx * radiusx);
        ray.normal.y = -ray.pos.y / (radiusy * radiusy);
        ray.normal.z = -ray.pos.z / (radiusz * radiusz);
        return;
    }

    ray.normal.x = 2 * (ray.pos.x - pos.x) / (radiusx * radiusx);
    ray.normal.y = 2 * (ray.pos.y - pos.y) / (radiusy * radiusy);
    ray.normal.z = 2 * (ray.pos.z - pos.z) / (radiusz * radiusz);
    ray.normal.normalise();

}

std::pair<Vector3, Vector3> Sphere::getBounds(const Ray &ray) const {

    Vector3 min(pos.x-radiusx,pos.y-radiusy,pos.z-radiusz);
    Vector3 max(pos.x+radiusx,pos.y+radiusy,pos.z+radiusz);

    return std::make_pair(min, max);

}



