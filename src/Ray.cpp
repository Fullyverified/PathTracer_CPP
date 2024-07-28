#include <Ray.h>
#include "Config.h"

#include <Sceneobject.h>

Ray::Ray(Vector3 origin, Vector3 dir) : origin(origin), dir(dir), pos(origin), hitpoint(0, 0, 0), normal(0, 0, 0),
                                       hit(false), sceneObject(nullptr) {
    luminanceRed.resize(config.bouncesPerRay + 1, std::vector<float>(4, 0));
    luminanceGreen.resize(config.bouncesPerRay + 1, std::vector<float>(4, 0));
    luminanceBlue.resize(config.bouncesPerRay + 1, std::vector<float>(4, 0));
}

Ray::Ray(Vector3 origin) : origin(origin), dir(0, 0, 0), pos(origin), hitpoint(0, 0, 0), normal(0, 0, 0), hit(false),
                          sceneObject(nullptr) {
    luminanceRed.resize(config.bouncesPerRay + 1, std::vector<float>(4, 0));
    luminanceGreen.resize(config.bouncesPerRay + 1, std::vector<float>(4, 0));
    luminanceBlue.resize(config.bouncesPerRay + 1, std::vector<float>(4, 0));
}

Ray::Ray():  origin(0,0,0), dir(0, 0, 0), pos(origin), hitpoint(0, 0, 0), normal(0, 0, 0), hit(false),
                          sceneObject(nullptr) {
    luminanceRed.resize(config.bouncesPerRay + 1, std::vector<float>(4, 0));
    luminanceGreen.resize(config.bouncesPerRay + 1, std::vector<float>(4, 0));
    luminanceBlue.resize(config.bouncesPerRay + 1, std::vector<float>(4, 0));
}

void Ray::march(const float &distance) {
    pos.setX(origin.getX() + (dir.getX() * distance));
    pos.setY(origin.getY() + (dir.getY() * distance));
    pos.setZ(origin.getZ() + (dir.getZ() * distance));

    /*pos.x = origin.x + (dir.x * distance);
    pos.y = origin.y + (dir.y * distance);
    pos.z = origin.z + (dir.z * distance);*/
}

void Ray::updateOrigin(const float &distance) {
    origin.setX(origin.getX() + (dir.getX() * distance));
    origin.setY(origin.getY() + (dir.getY() * distance));
    origin.setZ(origin.getZ() + (dir.getZ() * distance));

    /*origin.x = origin.x + (dir.x * distance);
    origin.y = origin.y + (dir.y * distance);
    origin.z = origin.z + (dir.z * distance);*/
    pos.set(origin);
}

void Ray::clearArray() {
    for (int j = 0; j < config.bouncesPerRay + 1; j++) {
        for (int i = 0; i < 4; i++) {
            luminanceRed[i][j] = 0;
            luminanceGreen[i][j] = 0;
            luminanceBlue[i][j] = 0;
        }
    }
}

void Ray::storeHitData(const int &currentBounce, const float &boolHit) {

    int pos = currentBounce + 1;
    luminanceRed[pos][0] = getHitObject()->getLum()[0]; // object brightness R
    luminanceRed[pos][1] = getNormal().dot(getDir()); // dot product of object normal and ray direction
    luminanceRed[pos][2] = getHitObject()->getCol()[0];
    luminanceRed[pos][3] = boolHit; // boolean hit

    luminanceGreen[pos][0] = getHitObject()->getLum()[1]; // object brightness R
    luminanceGreen[pos][1] = getNormal().dot(getDir()); // dot product of object normal and ray direction
    luminanceGreen[pos][2] = getHitObject()->getCol()[1];
    luminanceGreen[pos][3] = boolHit; // boolean hit

    luminanceBlue[pos][0] = getHitObject()->getLum()[2]; // object brightness R
    luminanceBlue[pos][1] = getNormal().dot(getDir()); // dot product of object normal and ray direction
    luminanceBlue[pos][2] = getHitObject()->getCol()[2];
    luminanceBlue[pos][3] = boolHit; // boolean hit
}

std::vector<float> Ray::sumHitData() {
    float redAmplitude = 0;
    float greenAmplitude = 0;
    float blueAmplitude = 0;
    for (int index = luminanceRed.size() - 1; index >= 0; index--) {
        if (luminanceRed[index][3] == hit) {
            redAmplitude = ((luminanceRed[index][0] + redAmplitude) * luminanceRed[index][1]) * luminanceRed[index][2];
        }
        if (luminanceGreen[index][3] == hit) {
            greenAmplitude = ((luminanceGreen[index][0] +  greenAmplitude) * luminanceGreen[index][1]) * luminanceGreen[index][2];
        }
        if (luminanceBlue[index][3] == hit) {
            blueAmplitude = ((luminanceBlue[index][0] + blueAmplitude) * luminanceBlue[index][1]) * luminanceBlue[index][2];
        }
    }
    return {redAmplitude, greenAmplitude, blueAmplitude};
}

void Ray::initialize(Ray &primaryRay) {
    getOrigin().set(primaryRay.getHitPoint());
    march(0);
    getHitPoint().set(primaryRay.getHitPoint());
    setHitObject(primaryRay.getHitObject());
    getDir().set(primaryRay.getDir());
}

Vector3 Ray::getPos() const { return pos; }

Vector3 Ray::getOrigin() const { return origin; }

Vector3 Ray::getDir() const { return dir; }

Vector3 Ray::getHitPoint() const { return hitpoint; }

Vector3 Ray::getNormal() const { return normal; }

void Ray::setHit(bool hit) { this->hit = hit; }

bool Ray::getHit() const { return hit; }

void Ray::setHitObject(SceneObject *hitObject) { this->sceneObject = hitObject; }

SceneObject* Ray::getHitObject() const { return sceneObject; }
