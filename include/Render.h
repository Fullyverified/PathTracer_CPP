#ifndef RENDER_H
#define RENDER_H

#include <atomic>
#include <vector>
#include <SceneObject.h>
#include <BVHNode.h>
#include <mutex>

#include "Camera.h"

class Render {

public:

Render();

    // bvh construction
    void constructBVHST(const std::vector<SceneObject*> &sceneObjectsList);
    void constructBVHMT(const std::vector<SceneObject*> &sceneObjectsList);
    void findBestPair(const std::vector<BVHNode *> &nodes, int start, int end, std::atomic<float> &globalBestCost, int &leftIndex, int &rightIndex, BVHNode *&bestLeft, BVHNode *&bestRight, std::mutex &mutex);
    //bvh performance profiling
    void BVHProilfing();

    void computePixels(std::vector<SceneObject*> &sceneobjectsList, Camera &cam, int &numRays, int &numBounces);
    void computePrimaryRay(Camera &cam, Ray &ray, int &x, int &y, BVHNode &rootNode);
    void computeSecondaryRay(Camera &cam, std::vector<std::vector<Ray>> &primaryRay, std::vector<std::vector<Ray>> &secondaryRay, int &x, int &y, BVHNode &rootNode);

    float lambertCosineLaw(Ray &ray, SceneObject* sceneObject);


    std::vector<BVHNode*> BVHNodes;

private:
};

#endif //RENDER_H
