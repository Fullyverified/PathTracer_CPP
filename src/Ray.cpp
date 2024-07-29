#include <Ray.h>
#include "Config.h"

#include <Sceneobject.h>

Ray::Ray(Vector3 origin, Vector3 dir) : origin(origin), dir(dir), pos(origin), hitpoint(0, 0, 0), normal(0, 0, 0),
hit(false), sceneObject(nullptr) {
}

Ray::Ray(Vector3 origin) : origin(origin), dir(0, 0, 0), pos(origin), hitpoint(0, 0, 0), normal(0, 0, 0), hit(false),
sceneObject(nullptr) {

}

Ray::Ray():  origin(0,0,0), dir(0, 0, 0), pos(origin), hitpoint(0, 0, 0), normal(0, 0, 0), hit(false),
sceneObject(nullptr) {
}

void Ray::march(const float &distance) {
    pos.setX(origin.getX() + (dir.getX() * distance));
    pos.setY(origin.getY() + (dir.getY() * distance));
    pos.setZ(origin.getZ() + (dir.getZ() * distance));
}

void Ray::updateOrigin(const float &distance) {
    origin.setX(origin.getX() + (dir.getX() * distance));
    origin.setY(origin.getY() + (dir.getY() * distance));
    origin.setZ(origin.getZ() + (dir.getZ() * distance));
    pos.set(origin);
}

void Ray::initialize(Ray &primaryRay) {
    getOrigin().set(primaryRay.getHitPoint());
    march(0);
    getHitPoint().set(primaryRay.getHitPoint());
    setHitObject(primaryRay.getHitObject());
    getDir().set(primaryRay.getDir());
}

Vector3 Ray::getPos() const { return pos; }

Vector3 Ray::getOrigin() const { return origin; }

Vector3 Ray::getDir() const { return dir; }

Vector3 Ray::getHitPoint() const { return hitpoint; }

Vector3 Ray::getNormal() const { return normal; }

void Ray::setHit(bool hit) { this->hit = hit; }

bool Ray::getHit() const { return hit; }

void Ray::setHitObject(SceneObject *hitObject) { this->sceneObject = hitObject; }

SceneObject* Ray::getHitObject() const { return sceneObject; }
