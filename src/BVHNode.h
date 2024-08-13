#ifndef BVHNODE_H
#define BVHNODE_H

#include "BoundingBox.h"
#include "SceneObject.h"

class BVHNode {
public:

    // two constructors
    BVHNode(BoundingBox* boundingBox, SceneObject& sceneObject);
    BVHNode(BoundingBox* boundingBox, BVHNode* left, BVHNode* right);
    ~BVHNode(); // deconstructor

    [[nodiscard]] int getNumChildren() const;
    [[nodiscard]] std::vector<float> getIntersectionDistance(Ray &ray) const;
    [[nodiscard]] BVHNode* searchBVHTree(Ray &ray);
    [[nodiscard]] float getArea() const;
    [[nodiscard]] BoundingBox* getBoundingBox() const;
    [[nodiscard]] SceneObject* getSceneObject() const;
    [[nodiscard]] BVHNode* getNodeLeft() const;
    [[nodiscard]] BVHNode* getNodeRight() const;

private:
    BVHNode *nodeLeft, *nodeRight;
    SceneObject *sceneObject;
    BoundingBox *boundingBox;
};


#endif //BVHNODE_H
