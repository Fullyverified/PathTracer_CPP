#ifndef RENDER_H
#define RENDER_H

#include <atomic>
#include <vector>
#include <SceneObject.h>
#include <BVHNode.h>
#include <mutex>

class Render {

public:

Render();

    void constructBVH(const std::vector<SceneObject*> &sceneObjectsList);

    void constructBVHMultiThreaded(const std::vector<SceneObject*> &sceneObjectsList);
    void findBestPair(const std::vector<BVHNode *> &nodes, int start, int end, std::atomic<float> &globalBestCost, int &leftIndex, int &rightIndex, BVHNode *&bestLeft, BVHNode *&bestRight, std::mutex &mutex);

    std::vector<BVHNode*> BVHNodes;

private:
};

#endif //RENDER_H
