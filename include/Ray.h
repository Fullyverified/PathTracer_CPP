#ifndef RAY_H
#define RAY_H
#include "Vector3.h"

class Ray {

public:
    Vector3 origin, pos, dir, hitpoint, normal;
    Ray(Vector3 origin, Vector3 dir);

    void marchRay(float& distance);

};


#endif //RAY_H
