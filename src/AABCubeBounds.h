#ifndef AABCUBEBOUNDS_H
#define AABCUBEBOUNDS_H

#include <random>

#include "SceneObject.h"
#include "Vector3.h"
#include "Ray.h"

class AABCubeBounds : public SceneObject {
public:
    AABCubeBounds(Vector3 minBounds, Vector3 maxBounds, Material* material);
    [[nodiscard]] Intersection getIntersectionDistance(Ray &ray) const override;
    Vector3 samplePoint (float r1, float r2) const override;
    void updateFaceWeights();
    [[nodiscard]] float getArea() const override;
    void computeArea() override;
    [[nodiscard]] Vector3 getPos() const override {return pos;}
    void setPos(Vector3 newPos) override {pos = newPos;}
    [[nodiscard]] Vector3 getDir() const override {return dir;}
    void setDir(Vector3 newDir) override {dir = newDir;}
    [[nodiscard]] Vector3 getScale() const override {return Vector3(1, 1, 1);}
    void setScale(Vector3 newScale) override {faceDist = std::discrete_distribution<int>(weights.begin(), weights.end());}
    [[nodiscard]] Material* getMaterial() const override {return material;}
    [[nodiscard]] Material* getMaterial(Ray& ray) const override {return material;}
    void setMaterial(Material* material) override {this->material = material;}
    void getNormal(Ray &ray) const override;
    [[nodiscard]] Vector3 getNormal(Vector3 sampledPoint) const override;
    [[nodiscard]] std::pair<Vector3, Vector3> getBounds() override;
    void printType() const override;
    std::string getType() const override;
    [[nodiscard]] int getObjID() const override;
    [[nodiscard]] bool isMesh() const override {return false;}
    [[nodiscard]] BVHNode* getMeshNode() const override {return nullptr;}

    Matrix4x4 getInvTransform() const override {return Matrix4x4();};

    AABCubeBounds(const AABCubeBounds& other) = delete; // disable copy constructor
    AABCubeBounds& operator=(const AABCubeBounds& other) = delete; // disable copy assignment=

private:
    Vector3 minBounds, maxBounds, size, pos, dir, colour, luminance;
    float area;
    Material* material;

    std::vector<float> weights; // for sampling faces uniformaly

    static thread_local std::mt19937 rng; // Thread-local RNG
    mutable std::discrete_distribution<int> faceDist;
};


#endif //AABCUBEBOUNDS_H
