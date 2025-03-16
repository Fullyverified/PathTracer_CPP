#ifndef VECTOR3_H
#define VECTOR3_H

#include <cmath>
#include <iostream>

class Vector3 {
public:
    Vector3() : x(0), y(0), z(0) {
    } // default constructor
    Vector3(const float &x, const float &y, const float &z) : x(x), y(y), z(z) {
    }

    Vector3(const float &value) : x(value), y(value), z(value) {
    }

    Vector3(const Vector3 &other) : x(other.getX()), y(other.getY()), z(other.getZ()) {
    }

    // adding
    [[nodiscard]] Vector3 operator+(const Vector3 &second) const {
        return {x + second.x, y + second.y, z + second.z};
    }

    Vector3 operator+(float scalar) {
        return {x + scalar, y + scalar, z + scalar};
    }

    Vector3 &operator+=(const Vector3 &second) {
        x += second.x;
        y += second.y;
        z += second.z;
        return *this;
    }

    Vector3 &operator+=(float scalar) {
        x += scalar;
        y += scalar;
        z += scalar;
        return *this;
    }

    // subtracting
    [[nodiscard]] Vector3 operator-(const Vector3 &second) const {
        return {x - second.x, y - second.y, z - second.z};
    }

    Vector3 operator-(float scalar) {
        return {x - scalar, y - scalar, z - scalar};
    }

    Vector3 operator-() const {
        return {-x, -y, -z};
    }

    Vector3 &operator-=(const Vector3 &second) {
        x -= second.x;
        y -= second.y;
        z -= second.z;
        return *this;
    }

    Vector3 &operator-=(float scalar) {
        x -= scalar;
        y -= scalar;
        z -= scalar;
        return *this;
    }

    // multiplying
    [[nodiscard]] Vector3 operator*(const Vector3 &second) const {
        return {x * second.x, y * second.y, z * second.z};
    }

    Vector3 operator*(float scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }

    Vector3 &operator*=(const Vector3 &second) {
        x *= second.x;
        y *= second.y;
        z *= second.z;
        return *this;
    }

    Vector3 &operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    // dividing
    [[nodiscard]] Vector3 operator/(const Vector3 &second) const {
        return {x / second.x, y / second.y, z / second.z};
    }

    Vector3 operator/(float scalar) const {
        return {x / scalar, y / scalar, z / scalar};
    }

    Vector3 &operator/=(const Vector3 &second) {
        x /= second.x;
        y /= second.y;
        z /= second.z;
        return *this;
    }

    Vector3 &operator/=(float scalar) {
        x /= scalar;
        y /= scalar;
        z /= scalar;
        return *this;
    }

    bool operator==(const Vector3& v1) const {
        return (x == v1.x && y == v1.y && z == v1.z);
    }

    // vector operations
    static float dot(const Vector3 &v1, const Vector3 &v2) {
        return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    }

    [[nodiscard]] float dot(const Vector3 &second) const {
        return x * second.x + y * second.y + z * second.z;
    }

    static float distance(const Vector3 &v1, const Vector3 &v2) {
        return sqrtf((v1.x - v2.x) * (v1.x - v2.x) + (v1.y - v2.y) * (v1.y - v2.y) + (v1.z - v2.z) * (v1.z - v2.z));
    }

    static float distanceSqrd(const Vector3 &v1, const Vector3 &v2) {
        return (v1.x - v2.x) * (v1.x - v2.x) + (v1.y - v2.y) * (v1.y - v2.y) + (v1.z - v2.z) * (v1.z - v2.z);
    }

    [[nodiscard]] Vector3 cross(const Vector3 &second) const {
        return {y * second.z - z * second.y, z * second.x - x * second.z, x * second.y - y * second.x};
    }

    Vector3 cross(const Vector3 &v1, const Vector3 &v2) {
        return {v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x};
    }

    float average() {
        float total = x + y + z;
        return total == 0 ? 0 : total / 3;
    }

    static float average(const Vector3& v1) {
        float total = v1.x + v1.y + v1.z;
        return total == 0 ? 0 : total / 3;
    }

    static float luminance(const Vector3& v1) {
        return (v1.x * 0.2126f) + (v1.y * 0.7152f) + (v1.z * 0.0722f) ;
    }

    void normalise() {
        const float magnitude = std::sqrt(x * x + y * y + z * z);
        if (magnitude == 0) return;
        x /= magnitude;
        y /= magnitude;
        z /= magnitude;
    }

    static void normalise(Vector3& v1) {
        const float magnitude = std::sqrt(v1.x * v1.x + v1.y * v1.y + v1.z * v1.z);
        if (magnitude == 0) return;
        v1.x /= magnitude;
        v1.y /= magnitude;
        v1.z /= magnitude;
    }

    Vector3 computeHalfVector(const Vector3 &viewDir, const Vector3 &lightDir) {
        Vector3 half = viewDir + lightDir;
        half.normalise();
        return half;
    }

    float lengthSquared() const {
        return x * x + y * y + z * z;
    }

    float length() const {
        return std::sqrtf(x * x + y * y + z * z);
    }

    static float length(const Vector3& v1) {
        return std::sqrtf(v1.x * v1.x + v1.y * v1.y + v1.z * v1.z);
    }

    static Vector3 reflect(const Vector3& wi, Vector3 &n) {
        float dotP = dot(wi, n);
        return Vector3(wi.x - n.x, wi.y - n.y, wi.z - n.z) * dotP * 2;
    }

    Vector3 reflect(const Vector3 &n) {
        float dotP = this->dot(n);
        return Vector3(x - n.x, y - n.y, z - n.z) * dotP * 2;
    }

    static Vector3 normalOfHalfAngle(const Vector3 &wo, const Vector3 &wi) {
        Vector3 h = wo + wi;
        h.normalise();
        return h;
    }

    // operator[]
    float &operator[](int index) {
        switch (index) {
            case 0: return x;
            case 1: return y;
            case 2: return z;
            default: throw std::out_of_range("Index out of range for Vector3");
        }
    }

    const float &operator[](int index) const {
        switch (index) {
            case 0: return x;
            case 1: return y;
            case 2: return z;
            default: throw std::out_of_range("Index out of range for Vector3");
        }
    }

    // generic functions

    float maxComponent() {
        return std::max(std::max(x, y), z);
    }

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

    void sanitize() {
        x = std::isfinite(x) ? x : 0.0f;
        y = std::isfinite(y) ? y : 0.0f;
        z = std::isfinite(z) ? z : 0.0f;
    }

    void print() const {
        std::cout << "x: " << x << ", y: " << y << ", z: " << z << std::endl;
    }

    void setX(float const &newX) { x = newX; }
    void setY(float const &newY) { y = newY; }
    void setZ(float const &newZ) { z = newZ; }

    [[nodiscard]] float getX() const { return x; }
    [[nodiscard]] float getY() const { return y; }
    [[nodiscard]] float getZ() const { return z; }

    float x, y, z;

private:
};

// Non-member operator function

inline Vector3 operator+(float scalar, const Vector3 &vec) {
    return {scalar + vec.x, scalar + vec.y, scalar + vec.z};
}

inline Vector3 operator-(float scalar, const Vector3 &vec) {
    return {scalar - vec.x, scalar - vec.y, scalar - vec.z};
}

inline Vector3 operator*(float scalar, const Vector3 &vec) {
    return {scalar * vec.x, scalar * vec.y, scalar * vec.z};
}

inline Vector3 operator/(float scalar, const Vector3 &vec) {
    return {scalar / vec.x, scalar / vec.y, scalar / vec.z};
}

#endif //VECTOR3_H
