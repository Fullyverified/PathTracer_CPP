#ifndef CAMERA_H
#define CAMERA_H
#include "Config.h"
#include "Vector3.h"

class Camera {
public:
    Camera(Vector3 pos, Vector3 dir);

    void upVector();

    void rightVector();

    void imagePlane();

    float toRadians(float &degrees) const;

    [[nodiscard]] Vector3 getPos() const;
    [[nodiscard]] Vector3 getDir() const;
    [[nodiscard]] Vector3 getRight() const;
    [[nodiscard]] Vector3 getUp() const;
    [[nodiscard]] float getPlaneWidth() const;
    [[nodiscard]] float getPlaneHeight() const;

    void initilizePitchYaw();
    void reInitilize();

    void moveForward(float elapsedTime);
    void moveBackward(float elapsedTime);
    void moveLeft(float elapsedTime);
    void moveRight(float elapsedTime);
    void moveUp(float elapsedTime);
    void moveDown(float elapsedTime);
    void updateDirection(float mouseX, float mouseY);

private:
    float planeWidth, planeHeight, aspectX, aspectY, yaw, pitch;
    Vector3 pos, dir, up, right;
};



#endif //CAMERA_H
