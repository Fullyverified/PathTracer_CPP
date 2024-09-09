#ifndef AABCUBECENTER_H
#define AABCUBECENTER_H

#include "Ray.h"
#include "SceneObject.h"
#include "Vector3.h"

class AABCubeCenter : public SceneObject {
public:
    AABCubeCenter(Vector3 pos, Vector3 length, float R, float G, float B, float RL, float GL, float BL, float roughness, float refrac, float transp);
    [[nodiscard]] bool objectCulling(Ray &ray) const override;
    std::pair<float, float> getIntersectionDistance(Ray &ray) const override;
    [[nodiscard]] bool intersectionCheck(Ray &ray) const override;
    [[nodiscard]] Vector3 getPos() const override;
    void getNormal(Ray &ray) const override;
    [[nodiscard]] std::pair<Vector3, Vector3> getBounds() const override;
    [[nodiscard]] Vector3 getCol() const override;
    [[nodiscard]] Vector3 getLum() const override;
    [[nodiscard]] float getRough() const override;
    [[nodiscard]] float getRefrac() const override;
    [[nodiscard]] float getTransp() const override;
    void printType() const override;
    [[nodiscard]] int getObjID() const override;

    AABCubeCenter(const AABCubeCenter& other) = delete; // disable copy constructor
    AABCubeCenter& operator=(const AABCubeCenter& other) = delete; // disable copy assignment

private:
    Vector3 pos, length, minBounds, maxBounds,  colour, luminance;;
    float roughness, refrac, transp;
};


#endif //AABCUBECENTER_H
