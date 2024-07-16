#ifndef BVHNODE_H
#define BVHNODE_H
#include "SceneObject.h"
#include "Boundingbox.h"

class BVHNode {
public:

    bool hitLeft, hitRight;
    BVHNode nodeLeft, nodeRight;
    SceneObject* sceneObject;
    BoundingBox boundingBox;

    // two constructors
    BVHNode (BoundingBox boundingBox, SceneObject* sceneObject);
    BVHNode (BoundingBox boundingBox, BVHNode left, BVHNode right);
    ~BVHNode ();

    // num children
    [nodiscard] int getNumChildren() const;

    //intersection distance
    [no discard] float getIntersectionDistance(const Ray &ray) const;

    // search bvh tree
    [no discard] BVHNode searchBVHTree(const Ray &ray);

private:
};


#endif //BVHNODE_H
