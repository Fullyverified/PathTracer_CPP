#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "Vector3.h"
#include "Vector2.h"

class Triangle {
public:
    Vector3 v0, v1, v2;
    Vector3 n0, n1, n2;
    Vector2 uv0, uv1, uv2;

    Triangle(const Vector3& v0, const Vector3& v1, const Vector3& v2, const Vector3& n0 = Vector3(), const Vector3& n1 = Vector3(), const Vector3& n2 = Vector3(), const Vector2& uv0 = Vector2(), const Vector2& uv1 = Vector2(), const Vector2& uv2 = Vector2())
        : v0(v0), v1(v1), v2(v2), n0(n0), n1(n1), n2(n2), uv0(uv0), uv1(uv1), uv2(uv2) {}


    [[nodiscard]] Vector3 getCentroid() const {return (v0 + v1 + v2) / 3.0f;}

};


#endif //TRIANGLE_H
