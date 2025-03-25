#ifndef VECTOR2_H
#define VECTOR2_H

#include <iostream>

class Vector2 {
public:
    Vector2() : x(0.0f), y(0.0f) {
    } // default constructor
    Vector2(const float &x, const float &y) : x(x), y(y) {
    }

    Vector2(const float &value) : x(value), y(value) {
    }

    Vector2(const Vector2 &other) : x(other.x), y(other.y) {
    }

    // adding
    [[nodiscard]] Vector2 operator+(const Vector2 &second) const {
        return {x + second.x, y + second.y};
    }

    Vector2 operator+(float scalar) {
        return {x + scalar, y + scalar};
    }

    Vector2 &operator+=(const Vector2 &second) {
        x += second.x;
        y += second.y;
        return *this;
    }

    Vector2 &operator+=(float scalar) {
        x += scalar;
        y += scalar;
        return *this;
    }

    // subtracting
    [[nodiscard]] Vector2 operator-(const Vector2 &second) const {
        return {x - second.x, y - second.y};
    }

    Vector2 operator-(float scalar) {
        return {x - scalar, y - scalar};
    }

    Vector2 operator-() const {
        return {-x, -y};
    }

    Vector2 &operator-=(const Vector2 &second) {
        x -= second.x;
        y -= second.y;
        return *this;
    }

    Vector2 &operator-=(float scalar) {
        x -= scalar;
        y -= scalar;
        return *this;
    }

    // multiplying
    [[nodiscard]] Vector2 operator*(const Vector2 &second) const {
        return {x * second.x, y * second.y};
    }

    Vector2 operator*(float scalar) const {
        return {x * scalar, y * scalar};
    }

    Vector2 &operator*=(const Vector2 &second) {
        x *= second.x;
        y *= second.y;
        return *this;
    }

    Vector2 &operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    // dividing
    [[nodiscard]] Vector2 operator/(const Vector2 &second) const {
        return {x / second.x, y / second.y};
    }

    Vector2 operator/(float scalar) const {
        return {x / scalar, y / scalar};
    }

    Vector2 &operator/=(const Vector2 &second) {
        x /= second.x;
        y /= second.y;
        return *this;
    }

    Vector2 &operator/=(float scalar) {
        x /= scalar;
        y /= scalar;
        return *this;
    }

    bool operator==(const Vector2& v1) const {
        return (x == v1.x && y == v1.y);
    }

    void print() const {
        std::cout << "x: " << x << ", y: " << y<< std::endl;
    }

    float x, y;
private:
};

#endif //VECTOR2_H