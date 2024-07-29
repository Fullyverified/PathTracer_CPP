#include <Camera.h>
#include <numbers>

Camera::Camera(Config& config, Vector3 pos, Vector3 dir) :
ISO(config.ISO), resX(config.resX), fOV(config.fOV), aspectX(config.aspectX), aspectY(config.aspectY), pos(pos), dir(dir), resY(0), up(0,0,0),
planeWidth(0), planeHeight(0), right(0,0,0) {
    resY = resX / (aspectX / aspectY);
    dir.normalise();
    upVector();
    up.normalise();
    rightVector();
    std::cout<<"Resolution: "<<resX<<" x "<<resY<<std::endl;
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


float Camera::getResX() const {
    return resX;
}

float Camera::getResY() const {
    return resY;
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