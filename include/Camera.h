#ifndef CAMERA_H
#define CAMERA_H
#include "Vector3.h"

class Camera {
public:
    Camera(float ISO, float resX, float fOV, float aspectX, float aspectY, Vector3 pos, Vector3 dir);

    void upVector();

    void rightVector();

    void imagePlane();

    float fOV, planeWidth, planeHeight, aspectX, aspectY, resX, resY, ISO;
    Vector3 pos, dir, up, right;

    float toRadians(float &degrees) const;


private:
};



#endif //CAMERA_H
