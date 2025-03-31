#ifndef SPHERE_H
#define SPHERE_H

#include "SceneObject.h"
#include "Vector3.h"
#include "Material.h"

class Sphere : public SceneObject {
public:
    Sphere(Vector3 pos, float radiusx, float radiusy, float radiusz, Material* material);
    Intersection getIntersectionDistance(Ray &ray) const override;
    Vector3 samplePoint (float r1, float r2) const override;
    [[nodiscard]] float getArea() const override;
    void computeArea() override;
    [[nodiscard]] Vector3 getPos() const override {return pos;}
    void setPos(Vector3 newPos) override {pos = newPos;}
    [[nodiscard]] Vector3 getDir() const override {return dir;}
    void setDir(Vector3 newDir) override {dir = newDir;}
    [[nodiscard]] Vector3 getScale() const override {return scale;}
    void setScale(Vector3 newScale) override {scale = newScale; computeArea();}
    void getNormal(Ray &ray) const override;
    [[nodiscard]] Vector3 getNormal(Vector3 sampledPos) const override;
    [[nodiscard]] std::pair<Vector3, Vector3> getBounds() override;
    [[nodiscard]] Material* getMaterial() const override {return material;}
    [[nodiscard]] Material* getMaterial(Ray& ray) const override {return material;}
    void setMaterial(Material* material) override {this->material = material;}
    void printType() const override;
    std::string getType() const override;
    [[nodiscard]] int getObjID() const override;
    [[nodiscard]] bool isMesh() const override {return false;}
    [[nodiscard]] BVHNode* getMeshNode() const override {return nullptr;}

    Matrix4x4 getInvTransform() const override {return Matrix4x4();};

    Sphere(const Sphere& other) = delete; // disable copy constructor
    Sphere& operator=(const Sphere& other) = delete; // disable copy assignment

private:
    Vector3 pos, dir, scale;
    float area;
    Material* material;
};

#endif //SPHERE_H
