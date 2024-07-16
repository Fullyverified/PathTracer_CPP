#include <Ray.h>

Ray::Ray(Vector3 origin, Vector3 dir):origin(origin),dir(dir), pos(0,0,0), hitpoint(0,0,0), normal(0,0,0) {
}

void Ray::marchRay(float& distance) {
    pos.x = origin.x + (dir.x * distance);
    pos.y = origin.y + (dir.y * distance);
    pos.z = origin.z + (dir.z * distance);
}

void Ray::updateOrigin(float &distance) {
    origin.x = origin.x + (dir.x * distance);
    origin.y = origin.y + (dir.y * distance);
    origin.z = origin.z + (dir.z * distance);
    pos.setV(origin);
}
