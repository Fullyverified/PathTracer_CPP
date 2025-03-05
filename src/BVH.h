#ifndef BVH_H
#define BVH_H

#include <mutex>

#include "SceneObject.h"
#include  "BVHNode.h"

struct BVHResult {
        int sceneObject;
        float close;
        float far;
    };

class BVH {
public:
    BVH() {}
    ~BVH() {
        for (auto node: BVHNodes) {
            delete node;
        }
        BVHNodes.clear();
    }

    // bvh logic
    void constructBVHST(const std::vector<SceneObject *> &sceneObjectsList);

    void constructBVHMT(const std::vector<SceneObject *> &sceneObjectsList);

    void findBestPair(const std::vector<BVHNode *> &nodes, int start, int end, std::atomic<float> &globalBestCost, int &leftIndex, int &rightIndex,
                      BVHNode *&bestLeft, BVHNode *&bestRight, std::mutex &mutex);

    void constructLinearBVH(const std::vector<SceneObject *> &sceneObjectsList);

    BVHResult searchLinearBVH(Ray &ray, const std::vector<SceneObject *> &sceneObjectsList) const;

    void updateSceneObjects(std::vector<SceneObject*> objects) {
        this->objects = objects;
    }

    std::vector<BVHNode *>& getBVHNodes() {
        return BVHNodes;
    }


private:
    std::vector<SceneObject*> objects;

    std::vector<BVHNode *> BVHNodes;


    // Unused
    struct LinearBVHNode {
        BoundingBox bounds;
        int leftChild;
        int rightChild;
        int objectIndex;
        bool isLeaf;
        int numChildren;
    };

    std::vector<LinearBVHNode> bvhNodes;
};

#endif //BVH_H
