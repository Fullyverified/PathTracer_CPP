#ifndef MESH_H
#define MESH_H

#include "SceneObject.h"
#include "Vector3.h"

class Mesh : public SceneObject {

    Mesh(Vector3 pos, Vector3 dir, float R, float G, float B, float RL, float GL, float BL, float roughness, float refrac, float transp);
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

    Mesh(const Mesh& other) = delete; // disable copy constructor
    Mesh& operator=(const Mesh& other) = delete; // disable copy assignment

private:
    Vector3 pos, dir, colour, luminance;
    float roughness, refrac, transp, R, G, B, RL, GL, BL;
};



#endif //MESH_H
