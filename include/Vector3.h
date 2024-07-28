#ifndef VECTOR3_H
#define VECTOR3_H

#include <cmath>
#include <immintrin.h>
#include <iostream>

class Vector3 {

public:
    Vector3() : x(0), y(0), z(0) {} // default constructor
    Vector3(const float &x, const float &y, const float &z) : x(x), y(y), z(z) {}
    Vector3(const Vector3 &other) : x(other.getX()), y(other.getY()), z(other.getZ()) {}

    // adding
    void add(const Vector3& second) {
        x += second.x;
        y += second.y;
        z += second.z;
    }

    /*void addSIMD(const Vector3& second) {
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
    }*/

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

    void flip() {
        x = -x;
        y = -y;
        z = -z;
    }

    void set(const float &newX, const float &newY, const float &newZ) {
        x = newX;
        y = newY;
        z = newZ;
    }

    void set(const Vector3 &other) {
        x = other.x;
        y = other.y;
        z = other.z;
    }

    void print() const {
        std::cout<<"x: "<<x<<", y: "<<y<<", z: "<<z<<std::endl;
    }

    void setX(float const &newX) {x = newX;}
    void setY(float const &newY) {y = newY;}
    void setZ(float const &newZ) {z = newZ;}

    [[nodiscard]] float getX() const {return x;}
    [[nodiscard]] float getY() const {return y;}
    [[nodiscard]] float getZ() const {return z;}

/*Vector3();
    Vector3(const float &x, const float &y, const float &z);
    Vector3(const Vector3 &other);

    // adding
    void add(const Vector3& second);

    /*void addSIMD(const Vector3& second) {
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
    }#1#

    [[nodiscard]] Vector3 addNew(const Vector3& second);

    // subtracting
    void subtract(const Vector3& second);

    [[nodiscard]] Vector3 subtractNew(const Vector3& second);

    [[nodiscard]] float dot(const Vector3& second);

    [[nodiscard]] Vector3 cross(const Vector3& second);

    void normalise();

    void flip();

    void set(const float &newX, const float &newY, const float &newZ);

    void set(const Vector3 &other);

    void print() const;

    void setX(float const &newX);
    void setY(float const &newY);
    void setZ(float const &newZ);

    [[nodiscard]] float getX() const;
    [[nodiscard]] float getY() const;
    [[nodiscard]] float getZ() const;*/

private:
    float x, y, z;
};

#endif //VECTOR3_H