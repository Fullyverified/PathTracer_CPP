#include <Ray.h>

Ray::Ray(Vector3 origin, Vector3 dir):origin(origin), dir(dir), pos(origin), hitpoint(0,0,0), normal(0,0,0) {
}

void Ray::marchRay(const float& distance) {

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

Vector3 Ray::getPos() const {
    return pos;
}

Vector3 Ray::getOrigin() const {
    return origin;
}

Vector3 Ray::getDir() const {
    return dir;
}

Vector3 Ray::getHitPoint() const {
    return hitpoint;
}

Vector3 Ray::getNormal() const {
    return normal;
}