#include <Ray.h>
#include "Config.h"

class SceneObject;

Ray::Ray(Vector3 origin, Vector3 dir):origin(origin), dir(dir), pos(origin), hitpoint(0,0,0), normal(0,0,0), hit(false), sceneObject(nullptr) {
    luminanceRed.resize(config.bouncesPerRay+1, std::vector<float*>(4, nullptr));
    luminanceGreen.resize(config.bouncesPerRay+1, std::vector<float*>(4, nullptr));
    luminanceBlue.resize(config.bouncesPerRay+1, std::vector<float*>(4, nullptr));
}

Ray::Ray(Vector3 origin):origin(origin), dir(0,0,0), pos(origin), hitpoint(0,0,0), normal(0,0,0), hit(false), sceneObject(nullptr) {
}

void Ray::march(const float& distance) {

    pos.setX(origin.getX() + (dir.getX() * distance));
    pos.setY(origin.getY() + (dir.getY() * distance));
    pos.setZ(origin.getZ() + (dir.getZ() * distance));

    /*pos.x = origin.x + (dir.x * distance);
    pos.y = origin.y + (dir.y * distance);
    pos.z = origin.z + (dir.z * distance);*/
}

void Ray::updateOrigin(const float &distance) {

    origin.setX(origin.getX() + (dir.getX() * distance));
    origin.setY(origin.getY() + (dir.getY() * distance));
    origin.setZ(origin.getZ() + (dir.getZ() * distance));

    /*origin.x = origin.x + (dir.x * distance);
    origin.y = origin.y + (dir.y * distance);
    origin.z = origin.z + (dir.z * distance);*/
    pos.setV(origin);
}

Vector3 Ray::getPos() const {return pos;}

Vector3 Ray::getOrigin() const {return origin;}

Vector3 Ray::getDir() const {return dir;}

Vector3 Ray::getHitPoint() const {return hitpoint;}

Vector3 Ray::getNormal() const {return normal;}

void Ray::setHit(bool hit) {this->hit = hit;}

bool Ray::getHit() const {return hit;}

void Ray::setHitObject(SceneObject* hitObject) {this->sceneObject = hitObject;}

SceneObject* Ray::getHitObject() const {return sceneObject;}