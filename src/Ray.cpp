#include "Ray.h"
#include "Config.h"

#include "SceneObject.h"

Ray::Ray(Vector3 origin, Vector3 dir) : dir(dir), pos(origin), normal(0, 0, 0),
                                        hit(false), sceneObject(nullptr), bCoords(0,0,0), internal(false) {
}

Ray::Ray(Vector3 origin) : dir(0, 0, 0), pos(origin), normal(0, 0, 0), hit(false),
sceneObject(nullptr), bCoords(0,0,0), internal(false) {

}

Ray::Ray(): dir(0, 0, 0), pos(0, 0, 0), normal(0, 0, 0), hit(false),
sceneObject(nullptr), bCoords(0,0,0), internal(false) {
}

void Ray::march(const float &distance) {
    pos.setX(pos.getX() + (dir.getX() * distance));
    pos.setY(pos.getY() + (dir.getY() * distance));
    pos.setZ(pos.getZ() + (dir.getZ() * distance));
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