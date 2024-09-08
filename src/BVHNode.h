#ifndef BVHNODE_H
#define BVHNODE_H

#include <unordered_map>

#include "BoundingBox.h"
#include "SceneObject.h"

class BVHNode {
public:
    struct traversalData {
        std::vector<float> bboxDistance;
        std::vector<float> objDistance;
        BVHNode* node;
    };

    // two constructors
    BVHNode(BoundingBox* boundingBox, SceneObject& sceneObject);
    BVHNode(BoundingBox* boundingBox, BVHNode* left, BVHNode* right);
    ~BVHNode(); // deconstructor

    [[nodiscard]] int getNumChildren() const;
    [[nodiscard]] std::vector<float> getIntersectionDistance(Ray &ray) const;
    [[nodiscard]] BVHNode* searchBVHTreeRoot(Ray &ray);
    [[nodiscard]] BVHNode* searchBVHTreeRecursive(Ray &ray, std::unordered_map<BVHNode*, traversalData>& cache);
    [[nodiscard]] BVHNode* searchBVHTreeOLD(Ray &ray);
    [[nodiscard]] float getArea() const;
    [[nodiscard]] BoundingBox* getBoundingBox() const;
    [[nodiscard]] SceneObject* getSceneObject() const;
    [[nodiscard]] BVHNode* getNodeLeft() const;
    [[nodiscard]] BVHNode* getNodeRight() const;

private:
    BVHNode *nodeLeft, *nodeRight;
    SceneObject *sceneObject;
    BoundingBox *boundingBox;
    int ID;
};


#endif //BVHNODE_H
