#ifndef VECTOR3_H
#define VECTOR3_H

#include <cmath>
#include <iostream>

class Vector3 {

public:
    Vector3() : x(0), y(0), z(0) {} // default constructor
    Vector3(const float &x, const float &y, const float &z) : x(x), y(y), z(z) {}
    Vector3(const float &value) : x(value), y(value), z(value) {}
    Vector3(const Vector3 &other) : x(other.getX()), y(other.getY()), z(other.getZ()) {}

    // adding
    [[nodiscard]] Vector3 operator+(const Vector3& second) const{
        return {x + second.x, y + second.y, z + second.z};
    }

    Vector3 operator+(float scalar) {
        return {x + scalar, y + scalar, z + scalar};
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

    // subtracting
    [[nodiscard]] Vector3 operator-(const Vector3& second) const{
        return {x - second.x, y - second.y, z - second.z};
    }

    Vector3 operator-(float scalar) {
        return {x - scalar, y - scalar, z - scalar};
    }

    // multiplying
    [[nodiscard]] Vector3 operator*(const Vector3& second) const {
        return {x * second.x, y * second.y, z * second.z};
    }

    Vector3 operator*(float scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }

    // dividing
    [[nodiscard]] Vector3 operator/(const Vector3& second) const {
        return {x / second.x, y / second.y, z / second.z};
    }

    Vector3 operator/(float scalar) const {
        return {x / scalar, y / scalar, z / scalar};
    }

    // vector operations
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

    // operator[]
    float& operator[](int index) {
        switch (index) {
            case 0 : return x;
            case 1 : return y;
            case 2 : return z;
            default: throw std::out_of_range("Index out of range for Vector3");
        }
    }

    const float& operator[](int index) const {
        switch (index) {
            case 0: return x;
            case 1: return y;
            case 2: return z;
            default: throw std::out_of_range("Index out of range for Vector3");
        }
    }

    // generic functions

    void flip() {
        x = -x;
        y = -y;
        z = -z;
    }

    void flipY() {
        y = -y;
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


private:
    float x, y, z;
};

#endif //VECTOR3_H