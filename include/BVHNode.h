#ifndef BVHNODE_H
#define BVHNODE_H
#include "SceneObject.h"
#include "BoundingBox.h"

class BVHNode {
public:

    BVHNode *nodeLeft, *nodeRight;
    SceneObject *sceneObject;
    BoundingBox *boundingBox;

    // two constructors
    BVHNode(BoundingBox* boundingBox, SceneObject& sceneObject);
    BVHNode(BoundingBox* boundingBox, BVHNode* left, BVHNode* right);
    ~BVHNode(); // deconstructor

    // num children
    [[nodiscard]] int getNumChildren() const;

    //intersection distance
    [[nodiscard]] float getIntersectionDistance(const Ray &ray) const;

    // search bvh tree
    [[nodiscard]] BVHNode* searchBVHTree(const Ray &ray);

    // get boundingbox's area
    [[nodiscard]] float getArea() const;

    // get boundingbox
    [[nodiscard]] BoundingBox* getBoundingBox() const;

private:
};


#endif //BVHNODE_H
