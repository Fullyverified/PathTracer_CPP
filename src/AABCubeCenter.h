#ifndef AABCUBECENTER_H
#define AABCUBECENTER_H

#include <random>

#include "Ray.h"
#include "SceneObject.h"
#include "Vector3.h"

class AABCubeCenter : public SceneObject {
public:
    AABCubeCenter(Vector3 pos, Vector3 length, Material* material);
    Intersection getIntersectionDistance(Ray &ray) const override;
    Vector3 samplePoint (float r1, float r2) const override;
    void updateFaceWeights();
    float getArea() const override;
    void computeArea() override;
    [[nodiscard]] Vector3 getPos() const override {return pos;}
    void setPos(Vector3 newPos) override {pos = newPos; updateBounds();}
    [[nodiscard]] Vector3 getDir() const override {return dir;}
    void setDir(Vector3 newDir) override {dir = newDir;}
    [[nodiscard]] Vector3 getScale() const override {return length;}
    void setScale(Vector3 newScale) override {length = newScale; updateBounds(); faceDist = std::discrete_distribution<int>(weights.begin(), weights.end());}
    [[nodiscard]] Material* getMaterial() const override {return material;}
    void setMaterial(Material* material) override {this->material = material;}
    void getNormal(Ray &ray) const override;
    [[nodiscard]] Vector3 getNormal(Vector3 pos) const override;
    [[nodiscard]] std::pair<Vector3, Vector3> getBounds() override;
    void printType() const override;
    std::string getType() const override;
    [[nodiscard]] int getObjID() const override;
    [[nodiscard]] bool isMesh() const override {return false;}
    [[nodiscard]] BVHNode* getMeshNode() const override {return nullptr;}




    void updateBounds();

    AABCubeCenter(const AABCubeCenter& other) = delete; // disable copy constructor
    AABCubeCenter& operator=(const AABCubeCenter& other) = delete; // disable copy assignment

private:
    Vector3 pos, dir, length, minBounds, maxBounds, size, colour, luminance;
    float area;
    Material* material;

    std::vector<float> weights; // for sampling faces uniformaly

    static thread_local std::mt19937 rng; // Thread-local RNG
    mutable std::discrete_distribution<int> faceDist;
};


#endif //AABCUBECENTER_H
