#ifndef VECTOR3_H
#define VECTOR3_H

#include <cmath>
#include <immintrin.h>

class Vector3 {

public:
    float x, y, z;
    Vector3() : x(0), y(0), z(0) {} // default constructor
    Vector3(const float x, const float y, const float z) : x(x), y(y), z(z) {}

    // adding
    void add(const Vector3& second) {
        x += second.x;
        y += second.y;
        z += second.z;
    }

    void addSIMD(const Vector3& second) {
        // load the first three components and pad the fourth with zero
        __m128 a = _mm_set_ps(0, z, y, x);
        __m128 b = _mm_set_ps(0, second.z, second.y, second.x);

        // perform addition
        __m128 c = _mm_add_ps(a, b);

        // store the results back
        // since we're using SSE which deals with 4 floats, but we need only 3
        _mm_store_ss(&x, c);
        _mm_store_ss(&y, _mm_shuffle_ps(c, c, _MM_SHUFFLE(0, 0, 0, 1)));
        _mm_store_ss(&z, _mm_shuffle_ps(c, c, _MM_SHUFFLE(0, 0, 0, 2)));
    }

    [[nodiscard]] Vector3 addNew(const Vector3& second) const{
        return {x + second.x, y + second.y, z + second.z};
    }

    // subtracting
    void subtract(const Vector3& second) {
        x -= second.x;
        y -= second.y;
        z -= second.z;
    }

    [[nodiscard]] Vector3 subtractNew(const Vector3& second) const{
        return {x - second.x, y - second.y, z - second.z};
    }

    [[nodiscard]] float dot(const Vector3& second) const {
        return x * second.x + y * second.y + z * second.z;
    }

    [[nodiscard]] Vector3 cross(const Vector3& second) const {
        return {y * second.z - z * second.y, z * second.x - x * second.z, x * second.y - y * second.x};
    }

    void normalise() {
        const float magnitude = std::sqrt(x * x + y * y + z * z);
        if (magnitude == 0) return;
        x /= magnitude;
        y /= magnitude;
        z /= magnitude;
    }

    void set(const float &newX, const float &newY, const float &newZ) {
        x = newX;
        y = newY;
        z = newZ;
    }

    void setV(const Vector3 &other) {
        x = other.x;
        y = other.y;
        z = other.z;
    }

    void setX(float const &newX) {x = newX;}
    void setY(float const &newY) {x = newY;}
    void setZ(float const &newZ) {x = newZ;}



private:

};

#endif //VECTOR3_H