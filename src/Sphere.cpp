#include <Sphere.h>
#include <Vector3.h>
#include <Ray.h>

Sphere::Sphere(Vector3 pos, float radius, float roughness, float refrac) :pos(pos), radius(radius), roughness(roughness), refrac(refrac){

}

bool Sphere::objectCulling(const Ray& ray) const override {

    const Vector3 centerOrigin = ray.pos.subtractNew(pos);
    const float scaleFactor = 1.0f / (radius * radius);

    // a should always = 1
    const float a = (ray.dir.x * ray.dir.x * scaleFactor) + (ray.dir.y * ray.dir.y * scaleFactor) + (ray.dir.z * ray.dir.z * scaleFactor);
    // b = 2 * (the dot product of the centerorigin vector by the direction vector)
    const float intermediary = (centerOrigin.x * ray.dir.x * scaleFactor) + (centerOrigin.y * ray.dir.y * scaleFactor) + (centerOrigin.z * ray.dir.z * scaleFactor);
    const float b = 2 * intermediary;
    // c = the dot product of centerorigin by itself, - the radius^2 of the sphere
    const float c = intermediary - 1;

    return false;
}