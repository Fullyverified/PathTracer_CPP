#ifndef RAY_H
#define RAY_H
#include "Vector3.h"

class Ray {

public:
    Ray(Vector3 origin, Vector3 dir);

    void marchRay(const float& distance);
    void updateOrigin(const float& distance);
    [[nodiscard]] Vector3 getPos() const;
    [[nodiscard]] Vector3 getOrigin() const;
    [[nodiscard]] Vector3 getDir() const;
    [[nodiscard]] Vector3 getHitPoint() const;
    [[nodiscard]] Vector3 getNormal() const;

private:
    Vector3 origin, pos, dir, hitpoint, normal;
};


#endif //RAY_H
