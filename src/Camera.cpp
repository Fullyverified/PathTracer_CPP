#include "Camera.h"
#include <numbers>

Camera::Camera(Vector3 pos, Vector3 dir) : aspectX(config.aspectX), aspectY(config.aspectY), pos(pos), dir(dir), worldUp(0,1,0),
planeWidth(0), planeHeight(0), right(0,0,0), up(0,0,0) {
    reInitilize();
    initilizePitchYaw();
}

void Camera::upVector() {

    up = dir.cross(right);
    up.normalise();

    if ((dir.getY() == 1 || dir.getY() == -1) && dir.getX() == 0 && dir.getZ() == 0) {
        up.setX(1);
        up.setY(0);
        up.setZ(0);
    }
}

void Camera::rightVector() {
    right = worldUp.cross(dir);
    right.normalise();
}

void Camera::imagePlane() {
    planeWidth = 2 * std::tan(toRadians(config.fOV) / 2) * 1;
    planeHeight = planeWidth / (aspectX / aspectY);
}

Vector3 Camera::getPos() const {return pos;}

Vector3 Camera::getDir() const {return dir;}

Vector3 Camera::getRight() const {return right;}

Vector3 Camera::getUp() const {return up;}

float Camera::getPlaneWidth() const {return planeWidth;}

float Camera::getPlaneHeight() const {return planeWidth;}

float Camera::toRadians(float &degrees) const{return (degrees * std::numbers::pi) / 180.0f;}

void Camera::reInitilize() {
    dir.normalise();
    rightVector();
    upVector();
    imagePlane();
}


void Camera::initilizePitchYaw() {
    yaw = std::atan2(dir.getZ(), dir.getX()) * 180.0f / std::numbers::pi; // Convert from radians to degrees
    pitch = std::asin(dir.getY()) * 180.0f / std::numbers::pi; // Convert from radians to degrees
}

void Camera::updateDirection(float mouseX, float mouseY) {
    yaw -= mouseX * config.mouseSensitivity;
    pitch -= mouseY * config.mouseSensitivity;

    // Clamp pitch to avoid gimbal lock (optional)
    const float maxPitch = 89.0f;
    if (pitch > maxPitch) pitch = maxPitch;
    if (pitch < -maxPitch) pitch = -maxPitch;

    // Normalize yaw to be within -180 to 180 degrees (optional)
    if (yaw > 180.0f) yaw -= 360.0f;
    if (yaw < -180.0f) yaw += 360.0f;

    float yawRad = toRadians(yaw);
    float pitchRad = toRadians(pitch);

    // Update forward vector based on yaw and pitch
    dir.setX(cos(yawRad) * cos(pitchRad));
    dir.setY(sin(pitchRad));
    dir.setZ(sin(yawRad) * cos(pitchRad));
    dir.normalise();

    // Update right and up vectors
    rightVector(); // Update right vector based on the new dir and up
    upVector();    // Update up vector if needed
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

void Camera::moveUp(float elapsedTime) {
    elapsedTime *= 0.10;
    pos.set(pos.getX() + up.getX() * elapsedTime,
        pos.getY() + up.getY() * elapsedTime,
        pos.getZ() + up.getZ() * elapsedTime);
}

void Camera::moveDown(float elapsedTime) {
    elapsedTime *= 0.10;
    pos.set(pos.getX() - up.getX() * elapsedTime,
        pos.getY() - up.getY() * elapsedTime,
        pos.getZ() - up.getZ() * elapsedTime);
}