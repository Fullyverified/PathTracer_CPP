#ifndef MESH_H
#define MESH_H

#include "SceneObject.h"
#include "Triangle.h"
#include "Vector3.h"
#include "LoadMesh.h"
#include "Matrix4x4.h"

class BVHNode;

class MeshObject : public SceneObject {
public:

    struct meshIntersection {
        BVHNode* node;
        float close;
        Triangle* triangle;
        Vector3 bcoords;
    };

    MeshObject(Vector3 pos, Vector3 dir, Vector3 scale, LoadMesh* mesh, Material* material);
    MeshObject(Vector3 pos, Vector3 dir, Vector3 scale, LoadMesh* mesh, std::vector<Material*> materials);
    Intersection getIntersectionDistance(Ray &ray) const override;
    Vector3 samplePoint (float r1, float r2) const override;
    float getArea() const override;
    void computeArea() override;
    [[nodiscard]] MeshObject::meshIntersection intersectTriangles(Ray &ray, BVHNode* leafNode) const;
    [[nodiscard]] Vector3 getPos() const override {return pos;}
    void setPos(Vector3 newPos) override {pos = newPos; updateMatrix();}
    [[nodiscard]] Vector3 getDir() const override {return dir;}
    void setDir(Vector3 newDir) override {dir = newDir; updateMatrix();}
    [[nodiscard]] Vector3 getScale() const override {return scale;}
    void setScale(Vector3 newScale) override {scale = newScale; updateMatrix();}
    [[nodiscard]] Material* getMaterial() const override {return materials[0];}
    [[nodiscard]] Material* getMaterial(Ray& ray) const override;
    void setMaterial(Material* material) override { materials[0] = material; setEmissiveTriangles(); computeArea();}
    void getNormal(Ray &ray) const override;
    [[nodiscard]] Vector3 getNormal(Vector3 sampledPos) const override;
    [[nodiscard]] std::pair<Vector3, Vector3> getBounds() override;
    void printType() const override;
    std::string getType() const override;
    [[nodiscard]] int getObjID() const override;
    [[nodiscard]] bool isMesh() const override {return true;}
    [[nodiscard]] BVHNode* getMeshNode() const override {return loadedMesh->getRootNode();}
    void setEmissiveTriangles();

    Matrix4x4 getInvTransform() const override {
        //return invTransform;
        return Matrix4x4();
    };

    void updateMatrix();

    MeshObject(const MeshObject& other) = delete; // disable copy constructor
    MeshObject& operator=(const MeshObject& other) = delete; // disable copy assignment

    struct Transform {
        const Vector3& pos;
        const Vector3& dir;
        const Vector3& scale;

        void rayToObj(Ray &ray) {
            ray.getPos().set(ray.getPos() - pos); // transform ray to object space
            ray.getPos().set(ray.getPos() / scale);
            ray.getDir().set(ray.getDir() / scale);
        }
    };

private:
    Vector3 pos, dir, scale;
    float area;
    //Material* material;
    std::vector<Material*> materials;
    std::pair<Vector3, Vector3> bounds;
    std::vector<Triangle*> triangles;
    std::vector<Triangle*> emissiveTriangles;
    LoadMesh* loadedMesh;

    //Matrix4x4 transform;
    //Matrix4x4 invTransform;
};



#endif //MESH_H
