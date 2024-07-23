#include <Camera.h>
#include <numbers>

Camera::Camera(float ISO, float resX, float fOV, float aspectX, float aspectY, Vector3 pos, Vector3 dir) :
ISO(ISO), resX(resX), fOV(fOV), aspectX(aspectX), aspectY(aspectY), pos(pos), dir(dir), resY(resX / (aspectX / aspectY)), up(0,0,0),
planeWidth(0), planeHeight(0), right(0,0,0) {

    dir.normalise();
    upVector();
    up.normalise();
    rightVector();
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

float Camera::toRadians(float &degrees) const{
    return (degrees * std::numbers::pi) / 180.0f;
}