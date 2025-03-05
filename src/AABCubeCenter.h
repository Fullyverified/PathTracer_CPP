#ifndef AABCUBECENTER_H
#define AABCUBECENTER_H

#include "Ray.h"
#include "SceneObject.h"
#include "Vector3.h"

class AABCubeCenter : public SceneObject {
public:
    AABCubeCenter(Vector3 pos, Vector3 length, Material* material);
    std::pair<float, float> getIntersectionDistance(Ray &ray) const override;
    [[nodiscard]] Vector3 getPos() const override {return pos;}
    void setPos(Vector3 newPos) override {pos = newPos; updateBounds();}
    [[nodiscard]] Vector3 getDir() const override {return dir;}
    void setDir(Vector3 newDir) override {dir = newDir;}
    [[nodiscard]] Vector3 getScale() const override {return length;}
    void setScale(Vector3 newScale) override {length = newScale; updateBounds();}
    [[nodiscard]] Material* getMaterial() const override {return material;}
    void setMaterial(Material* material) override {this->material = material;}
    void getNormal(Ray &ray) const override;
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
    Vector3 pos, dir, length, minBounds, maxBounds,  colour, luminance;;
    Material* material;
};


#endif //AABCUBECENTER_H
