#ifndef RENDER_H
#define RENDER_H

#include <atomic>
#include <vector>
#include <SceneObject.h>
#include <BVHNode.h>
#include <mutex>
#include <random>

#include "Camera.h"
#include "Config.h"

class Render {

public:
    Config config;

    Render(const Config& config, Camera &cam);
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
    float lambertCosineLaw(Ray &ray, SceneObject* sceneObject) const;
    float sumHitDataRGB(std::vector<std::vector<float>> vector, int &currentBounce, float &dotProduct, float &objectBrightness, float &objectReflectivity, float &boolHit) const;

    // multithreading logic
    // multithreading logic
    std::pair<int, int> primarySegments(float resI, int &numThreads, std::pair<int, int>, int i);
    std::pair<int, int> secondarySegment(float resI, int &numThreads, std::pair<int, int>, int i);

    // cleanup
    void intialiseObjects();
    void deleteObjects();

private:
    std::vector<BVHNode*> BVHNodes;
    std::vector<std::vector<float*>> avgR, avgG, avgB, absR, absG, absB; // avg is abs divided by number of rays for each iteration - abs is not
    std::vector<std::vector<Ray*>> primaryRay, secondaryRay;

    float primaryRayStep, secondaryRayStep;
    int numRays, numBounces, resX, resY, frameTime;
    Camera &cam;
    std::pair<int, int> boundsX;
    std::pair<int, int> boundsY;

    static thread_local std::mt19937 rng;  // Thread-local RNG
    mutable std::uniform_real_distribution<float> dist;
    float pi = 3.14159265358979323846f;
};

#endif //RENDER_H
