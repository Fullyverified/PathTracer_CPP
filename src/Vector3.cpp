#include "Vector3.h"

// this code is now inlined. 4.8x faster
/*Vector3::Vector3() : x(0), y(0), z(0) {
} // default constructor
Vector3::Vector3(const float &x, const float &y, const float &z) : x(x), y(y), z(z) {
}

Vector3::Vector3(const Vector3 &other) : x(other.getX()), y(other.getY()), z(other.getZ()) {
}

// adding
void Vector3::add(const Vector3 &second) {
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
}#1#

Vector3 Vector3::addNew(const Vector3 &second) {
    return {x + second.x, y + second.y, z + second.z};
}

// subtracting
void Vector3::subtract(const Vector3 &second) {
    x -= second.x;
    y -= second.y;
    z -= second.z;
}

Vector3 Vector3::subtractNew(const Vector3 &second) {
    return {x - second.x, y - second.y, z - second.z};
}

float Vector3::dot(const Vector3 &second) {
    return x * second.x + y * second.y + z * second.z;
}

Vector3 Vector3::cross(const Vector3 &second) {
    return {y * second.z - z * second.y, z * second.x - x * second.z, x * second.y - y * second.x};
}

void Vector3::normalise() {
    const float magnitude = std::sqrt(x * x + y * y + z * z);
    if (magnitude == 0) return;
    x /= magnitude;
    y /= magnitude;
    z /= magnitude;
}

void Vector3::flip() {
    x = -x;
    y = -y;
    z = -z;
}

void Vector3::set(const float &newX, const float &newY, const float &newZ) {
    x = newX;
    y = newY;
    z = newZ;
}

void Vector3::set(const Vector3 &other) {
    x = other.x;
    y = other.y;
    z = other.z;
}

void Vector3::print() const {
    std::cout << "x: " << x << ", y: " << y << ", z: " << z << std::endl;
}

void Vector3::setX(float const &newX) { x = newX; }
void Vector3::setY(float const &newY) { y = newY; }
void Vector3::setZ(float const &newZ) { z = newZ; }

float Vector3::getX() const { return x; }
float Vector3::getY() const { return y; }
float Vector3::getZ() const { return z; }*/
