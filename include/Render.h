#ifndef RENDER_H
#define RENDER_H

#include <atomic>
#include <vector>
#include <mutex>
#include <random>

#include <SceneObject.h>
#include <BVHNode.h>
#include "Camera.h"

class Render {

public:
    Render(Camera &cam);
    ~Render() = default;

    void computePixels(std::vector<SceneObject*> &sceneobjectsList, Camera &cam);

    // bvh logic
    void constructBVHST(const std::vector<SceneObject*> &sceneObjectsList);

    void constructBVHMT(const std::vector<SceneObject*> &sceneObjectsList);
    void findBestPair(const std::vector<BVHNode *> &nodes, int start, int end, std::atomic<float> &globalBestCost, int &leftIndex, int &rightIndex, BVHNode *&bestLeft, BVHNode *&bestRight, std::mutex &mutex);

    void BVHProilfing();

    // traversal logic
    void computePrimaryRay(Camera &cam, std::vector<std::vector<Ray*>> &primaryRay, int xstart, int xend, int ystart, int yend, BVHNode &rootNode, std::mutex &mutex) const;
    void computeSecondaryRay(Camera &cam, std::vector<std::vector<Ray*>> &primaryRayV, std::vector<std::vector<Ray*>> &secondaryRayV, int xstart, int xend, int ystart, int yend, BVHNode &rootNode, std::mutex &mutex) const;

    // bounce logic
    void sampleReflectionDirection(Ray &ray, SceneObject &sceneObject, bool flipNormal) const;
    void sampleRefractionDirection(Ray &ray, SceneObject &sceneObject, bool flipNormal) const;

    // multithreading logic
    std::pair<int, int> secondarySegments(float resI, int &numThreads, std::pair<int, int>, int i);

    // cleanup
    void intialiseObjects();
    void deleteObjects();

    void printScreen();


private:
    std::vector<BVHNode*> BVHNodes;
    mutable std::vector<std::vector<float>> lumR, lumG, lumB; // mutable - no two threads will ever rw the same index
    std::vector<std::vector<Ray*>> primaryRay, secondaryRay;

    float primaryRayStep, secondaryRayStep;
    int resX, resY;
    Camera &cam;
    std::pair<int, int> boundsX;
    std::pair<int, int> boundsY;

    static thread_local std::mt19937 rng;  // Thread-local RNG
    mutable std::uniform_real_distribution<float> dist;
    float pi = 3.14159265358979323846f;
};

#endif //RENDER_H
