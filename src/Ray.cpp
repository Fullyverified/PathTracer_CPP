#include "Ray.h"
#include "Config.h"

#include "SceneObject.h"

Ray::Ray(Vector3 origin, Vector3 dir) : origin(origin), dir(dir), pos(origin), hitpoint(0, 0, 0), normal(0, 0, 0),
                                        hit(false), sceneObject(nullptr), bCoords(0,0,0) {
}

Ray::Ray(Vector3 origin) : origin(origin), dir(0, 0, 0), pos(origin), hitpoint(0, 0, 0), normal(0, 0, 0), hit(false),
sceneObject(nullptr), bCoords(0,0,0) {

}

Ray::Ray():  origin(0,0,0), dir(0, 0, 0), pos(origin), hitpoint(0, 0, 0), normal(0, 0, 0), hit(false),
sceneObject(nullptr), bCoords(0,0,0) {
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
    origin.set(primaryRay.getHitPoint());
    pos.set(primaryRay.getHitPoint());
    hitpoint.set(primaryRay.getHitPoint());
    sceneObject = primaryRay.getHitObject();
    dir = primaryRay.getDir();
    hit = primaryRay.getHit();
}

//Vector3& Ray::getPos() { return pos; }

//Vector3& Ray::getOrigin() { return origin; }

//Vector3& Ray::getDir() { return dir; }

//Vector3& Ray::getHitPoint() { return hitpoint; }

//Vector3& Ray::getNormal() { return normal; }

//void Ray::setHit(bool hit) { this->hit = hit; }

//bool Ray::getHit() const { return hit; }

//void Ray::setHitObject(SceneObject *hitObject) { this->sceneObject = hitObject; }

//SceneObject* Ray::getHitObject() const { return sceneObject; }
