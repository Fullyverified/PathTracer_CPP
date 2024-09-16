#ifndef MESH_H
#define MESH_H

#include "SceneObject.h"
#include "Triangle.h"
#include "Vector3.h"
#include "LoadMesh.h"

class MeshObject : public SceneObject {
public:
    MeshObject(Vector3 pos, Vector3 dir, Vector3 scale, LoadMesh& mesh, float R, float G, float B, float RL, float GL, float BL, float roughness, float refrac, float transp, bool flipY);
    std::pair<float, float> getIntersectionDistance(Ray &ray) const override;
    [[nodiscard]] Vector3 getPos() const override;
    void getNormal(Ray &ray) const override;
    [[nodiscard]] std::pair<Vector3, Vector3> getBounds() override;
    [[nodiscard]] Vector3 getCol() const override;
    [[nodiscard]] Vector3 getLum() const override;
    [[nodiscard]] float getRough() const override;
    [[nodiscard]] float getRefrac() const override;
    [[nodiscard]] float getTransp() const override;
    void printType() const override;
    [[nodiscard]] int getObjID() const override;

    MeshObject(const MeshObject& other) = delete; // disable copy constructor
    MeshObject& operator=(const MeshObject& other) = delete; // disable copy assignment

private:
    Vector3 pos, dir, scale, colour, luminance;
    std::pair<Vector3, Vector3> bounds;
    float roughness, refrac, transp;
    bool flipY;
    std::vector<Triangle*> triangles;
    LoadMesh &loadedMesh;
};



#endif //MESH_H
