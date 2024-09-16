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
    [[nodiscard]] std::pair<float, float> getIntersectionDistance(Ray &ray) const;
    [[nodiscard]] std::pair<BVHNode*, float> searchBVHTree(Ray &ray);
    [[nodiscard]] std::pair<BVHNode*, float> searchBVHTreeTest(Ray &ray, long &numIterations);
    [[nodiscard]] float getArea() const;

    [[nodiscard]] BoundingBox* getBoundingBox() const;
    [[nodiscard]] SceneObject* getSceneObject() const;
    [[nodiscard]] BVHNode* getNodeLeft() const;
    [[nodiscard]] BVHNode* getNodeRight() const;

    void setLeaf(bool leaf);
    [[nodiscard]] bool getLeaf();

private:
    BVHNode *nodeLeft, *nodeRight;
    SceneObject *sceneObject;
    BoundingBox *boundingBox;
    bool isLeaf;
};

#endif //BVHNODE_H
