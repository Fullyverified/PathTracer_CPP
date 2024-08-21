#ifndef RENDER_H
#define RENDER_H

#include <atomic>
#include <vector>
#include <mutex>
#include <random>

#include "SceneObject.h"
#include "BVHNode.h"
#include "Camera.h"
#include "Ray.h"
#include "SDLWindow.h"

class Render {

public:
    Render(Camera &cam);
    ~Render() = default;

    // render loop
    void renderLoop(std::vector<SceneObject*> &sceneobjectsList, Camera &cam, SDLWindow &window);
    void computePixels(std::vector<SceneObject*> &sceneobjectsList, Camera &cam);

    // bvh logic
    void constructBVHST(const std::vector<SceneObject*> &sceneObjectsList);

    void constructBVHMT(const std::vector<SceneObject*> &sceneObjectsList);
    void findBestPair(const std::vector<BVHNode *> &nodes, int start, int end, std::atomic<float> &globalBestCost, int &leftIndex, int &rightIndex, BVHNode *&bestLeft, BVHNode *&bestRight, std::mutex &mutex);

    void BVHProfiling();

    // traversal logic
    void computePrimaryRay(Camera &cam, int xstart, int xend, int ystart, int yend, BVHNode &rootNode, std::mutex &mutex) const;
    void computeSecondaryRay(int xstart, int xend, int ystart, int yend, BVHNode &rootNode, std::mutex &mutex) const;

    // bounce logic
    void sampleReflectionDirection(Ray &ray, SceneObject &sceneObject, bool flipNormal) const;
    void sampleRefractionDirection(Ray &ray, SceneObject &sceneObject, bool flipNormal) const;

    // multithreading logic
    std::pair<int, int> threadSegments(float resI, int &numThreads, std::pair<int, int>, int i);

    // tone mapping & draw screen
    void toneMap();

    // cleanup
    void intialiseObjects();
    void deleteObjects();

    void printScreenASCII();


private:
    std::vector<BVHNode*> BVHNodes;
    mutable std::vector<float> lumR, lumG, lumB; // mutable - no two threads will ever rw the same index
    mutable std::vector<float> absR, absG, absB;
    mutable std::vector<Ray*> primaryRay, secondaryRay;
    uint8_t* pixels;

    float primaryRayStep, secondaryRayStep;
    int resX, resY, iterations, numThreads;
    Camera &cam;
    std::pair<int, int> boundsX;
    std::pair<int, int> boundsY;

    bool running;
    static thread_local std::mt19937 rng;  // Thread-local RNG
    mutable std::uniform_real_distribution<float> dist;
    float pi = 3.14159265358979323846f;
};

#endif //RENDER_H
