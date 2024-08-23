#include "Camera.h"
#include <numbers>

Camera::Camera(Config& config, Vector3 pos, Vector3 dir) :
ISO(config.ISO), fOV(config.fOV), aspectX(config.aspectX), aspectY(config.aspectY), pos(pos), dir(dir), up(0,1,0),
planeWidth(0), planeHeight(0), right(0,0,0) {
    dir.normalise();
    upVector();
    up.normalise();
    rightVector();
    imagePlane();
}

void Camera::upVector() {
    if ((dir.getY() == 1 || dir.getY() == -1) && dir.getX() == 0 && dir.getZ() == 0) {
        up.setX(1);
        up.setY(0);
        up.setZ(0);
    }
}

void Camera::rightVector() {
    // calculate right vector
    right.setX((up.getY() * dir.getZ()) - (up.getZ() * dir.getY()));
    right.setY((up.getZ() * dir.getX()) - (up.getX() * dir.getZ()));
    right.setZ((up.getX() * dir.getY()) - (up.getY() * dir.getX()));
    right.normalise();
}

void Camera::imagePlane() {
    planeWidth = 2 * std::tan(toRadians(fOV) / 2) * 1;
    planeHeight = planeWidth / (aspectX / aspectY);
}

Vector3 Camera::getPos() const {
    return pos;
}

Vector3 Camera::getDir() const {
    return dir;
}

Vector3 Camera::getRight() const {
    return right;
}

Vector3 Camera::getUp() const {
    return up;
}

float Camera::getPlaneWidth() const {
    return planeWidth;
}

float Camera::getPlaneHeight() const {
    return planeWidth;
}

float Camera::toRadians(float &degrees) const{
    return (degrees * std::numbers::pi) / 180.0f;
}

void Camera::moveForward(float elapsedTime) {
    elapsedTime *= 0.10;
    pos.set(pos.getX() + dir.getX() * elapsedTime,
        pos.getY() + dir.getY() * elapsedTime,
        pos.getZ() + dir.getZ() * elapsedTime);
}

void Camera::moveBackward(float elapsedTime) {
    elapsedTime *= -0.10;
    pos.set(pos.getX() + dir.getX() * elapsedTime,
        pos.getY() + dir.getY() * elapsedTime,
        pos.getZ() + dir.getZ() * elapsedTime);
}

void Camera::moveLeft(float elapsedTime) {
    elapsedTime *= -0.10;
    pos.set(pos.getX() + right.getX() * elapsedTime,
        pos.getY() + right.getY() * elapsedTime,
        pos.getZ() + right.getZ() * elapsedTime);
}

void Camera::moveRight(float elapsedTime) {
    elapsedTime *= 0.10;
    pos.set(pos.getX() + right.getX() * elapsedTime,
        pos.getY() + right.getY() * elapsedTime,
        pos.getZ() + right.getZ() * elapsedTime);
}