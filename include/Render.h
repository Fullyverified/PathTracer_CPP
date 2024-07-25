#ifndef RENDER_H
#define RENDER_H

#include <atomic>
#include <vector>
#include <SceneObject.h>
#include <BVHNode.h>
#include <mutex>

#include "Camera.h"
#include "Config.h"

class Render {

public:
    Config config;

    Render(const Config& config, const Camera& cam);
    ~Render() = default;

    // bvh construction
    void constructBVHST(const std::vector<SceneObject*> &sceneObjectsList);

    void constructBVHMT(const std::vector<SceneObject*> &sceneObjectsList);
    void findBestPair(const std::vector<BVHNode *> &nodes, int start, int end, std::atomic<float> &globalBestCost, int &leftIndex, int &rightIndex, BVHNode *&bestLeft, BVHNode *&bestRight, std::mutex &mutex);

    void BVHProilfing();

    // render logic
    void computePixels(std::vector<SceneObject*> &sceneobjectsList, Camera &cam);

    void computePrimaryRay(Camera &cam, std::vector<std::vector<Ray*>> &primaryRay, int xstart, int xend, int ystart, int yend, BVHNode rootNode, std::mutex &mutex) const;
    void computeSecondaryRay(Camera &cam, std::vector<std::vector<Ray>> &primaryRayV, std::vector<std::vector<Ray>> &secondaryRayV, BVHNode &rootNode) const;

    void cosineWeightedHemisphereImportanceSampling(Ray &ray, SceneObject* &sceneObject, bool flipNormal);
    void refractionDirection(Ray &ray, SceneObject* &sceneObject);

    float lambertCosineLaw(Ray &ray, SceneObject* sceneObject);

    std::pair<int, int> threadedRenderSegmentation(float resI, int &numThreads, std::pair<int, int>, int i);

    void intialiseObjects();
    void deleteObjects();

private:
    std::vector<BVHNode*> BVHNodes;
    std::vector<std::vector<float*>> avgR, avgG, avgB, absR, absG, absB; // avg is abs divided by number of rays for each iteration - abs is not
    std::vector<std::vector<Ray*>> primaryRay, secondaryRay;

    float primaryRayStep, secondaryRayStep;
    int numRays, numBounces, resX, resY, frameTime;
    Camera cam;
};

#endif //RENDER_H
