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

    [[nodiscard]] Vector3 getPos() const {return pos;}
    [[nodiscard]] Vector3 getDir() const {return dir;}
    [[nodiscard]] Vector3 getRight() const {return right;}
    [[nodiscard]] Vector3 getUp() const {return up;}
    [[nodiscard]] float getPlaneWidth() const {return planeWidth;}
    [[nodiscard]] float getPlaneHeight() const {return planeWidth;}

    void initilizePitchYaw();
    void reInitilize();

    void moveForward(float elapsedTime);
    void moveBackward(float elapsedTime);
    void moveLeft(float elapsedTime);
    void moveRight(float elapsedTime);
    void moveUp(float elapsedTime);
    void moveDown(float elapsedTime);
    void updateDirection(float mouseX, float mouseY);

    bool& getCamMoved() {
        return camMoved;
    }

private:
    float planeWidth, planeHeight, aspectX, aspectY, yaw, pitch;
    Vector3 pos, dir, up, right, worldUp;
    bool camMoved;
    float sensitivity = 10;
};



#endif //CAMERA_H
