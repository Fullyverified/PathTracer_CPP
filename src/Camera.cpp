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
    if (dir.y == 1 || dir.y == -1 && dir.x == 0 && dir.z == 0) {
        up.x = 1;
        up.y = 0;
        up.z = 0;
    }
}

void Camera::rightVector() {
    // calculate right vector
    right.x = (up.y * dir.z) - (up.z * dir.y);
    right.y = (up.z * dir.x) - (up.x * dir.z);
    right.z = (up.x * dir.y) - (up.y * dir.x);
    right.normalise();
}

void Camera::imagePlane() {
    planeWidth = 2 * std::tan(toRadians(fOV) / 2) * 1;
    planeHeight = planeWidth / (aspectX / aspectY);
}

float Camera::toRadians(float &degrees) const{
    return (degrees * std::numbers::pi) / 180.0f;
}