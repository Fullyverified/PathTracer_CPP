#ifndef CAMERA_H
#define CAMERA_H
#include "Config.h"
#include "Vector3.h"

class Camera {
public:

    Config config;

    Camera(Config& config, Vector3 pos, Vector3 dir);

    void upVector();

    void rightVector();

    void imagePlane();

    float toRadians(float &degrees) const;

    [[nodiscard]] float getResX() const;
    [[nodiscard]] float getResY() const;
    [[nodiscard]] Vector3 getPos() const;
    [[nodiscard]] Vector3 getDir() const;
    [[nodiscard]] Vector3 getRight() const;
    [[nodiscard]] Vector3 getUp() const;
    [[nodiscard]] float getPlaneWidth() const;
    [[nodiscard]] float getPlaneHeight() const;


private:
    float fOV, planeWidth, planeHeight, aspectX, aspectY, resX, resY, ISO;
    Vector3 pos, dir, up, right;
};



#endif //CAMERA_H
