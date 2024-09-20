#ifndef MESH_H
#define MESH_H

#include "SceneObject.h"
#include "Triangle.h"
#include "Vector3.h"
#include "LoadMesh.h"

class MeshObject : public SceneObject {
public:

    struct meshIntersection {
        BVHNode* node;
        float close;
        float far;
        Triangle* triangle;
        Vector3 bcoords;
    };

    MeshObject(Vector3 pos, Vector3 dir, Vector3 scale, LoadMesh& mesh, float R, float G, float B, float RL, float GL, float BL, float roughness, float refrac, float transp);
    std::pair<float, float> getIntersectionDistance(Ray &ray) const override;
    [[nodiscard]] MeshObject::meshIntersection intersectTriangles(Ray &ray, BVHNode* leafNode) const;
    [[nodiscard]] Vector3 getPos() const override;
    [[nodiscard]] Vector3 getScale() const override {return scale;}
    void getNormal(Ray &ray) const override;
    [[nodiscard]] std::pair<Vector3, Vector3> getBounds() override;
    [[nodiscard]] Vector3 getCol() const override;
    [[nodiscard]] Vector3 getLum() const override;
    [[nodiscard]] float getRough() const override;
    [[nodiscard]] float getRefrac() const override;
    [[nodiscard]] float getTransp() const override;
    void printType() const override;
    [[nodiscard]] int getObjID() const override;
    [[nodiscard]] bool isMesh() const override {return true;}
    [[nodiscard]] BVHNode* getMeshNode() const override {return loadedMesh.getRootNode();}

    MeshObject(const MeshObject& other) = delete; // disable copy constructor
    MeshObject& operator=(const MeshObject& other) = delete; // disable copy assignment

    struct Transform {
        const Vector3& pos;
        const Vector3& dir;
        const Vector3& scale;
        const MeshObject* meshObject;

        void rayToObj(Ray &ray) {
            ray.getPos().set(ray.getPos() - pos); // transform ray to object space
            ray.getPos().set(ray.getPos() / scale);
            ray.getDir().set(ray.getDir() / scale);
        }
        void rayToWorld(Ray &ray) {
            ray.getDir().set(ray.getDir() * scale);
            ray.getPos().set(ray.getPos() * scale);
            ray.getPos().set(ray.getPos() + pos); // transform ray to world space
        }
    };

private:
    Vector3 pos, dir, scale, colour, luminance;
    std::pair<Vector3, Vector3> bounds;
    float roughness, refrac, transp;
    std::vector<Triangle*> triangles;
    LoadMesh &loadedMesh;
};



#endif //MESH_H
