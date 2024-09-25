#ifndef RENDER_H
#define RENDER_H

#include <atomic>
#include <vector>
#include <mutex>
#include <random>
#include <stack>

#include "BoundingBox.h"
#include "SceneObject.h"
#include "BVHNode.h"
#include "Camera.h"
#include "Ray.h"
#include "SDLWindow.h"

class Render {
public:
    struct BVHResult {
        int sceneObject;
        float close;
        float far;
    };

    Render(Camera &cam);

    ~Render() = default;

    // render loop
    void renderLoop(std::vector<SceneObject *> &sceneobjectsList, SDLWindow &window);

    void computePixels(std::vector<SceneObject *> &sceneobjectsList);

    // bvh logic
    void constructBVHST(const std::vector<SceneObject *> &sceneObjectsList);

    void constructBVHMT(const std::vector<SceneObject *> &sceneObjectsList);

    void findBestPair(const std::vector<BVHNode *> &nodes, int start, int end, std::atomic<float> &globalBestCost, int &leftIndex, int &rightIndex,
                      BVHNode *&bestLeft, BVHNode *&bestRight, std::mutex &mutex);

    void constructLinearBVH(const std::vector<SceneObject *> &sceneObjectsList);

    BVHResult searchLinearBVH(Ray &ray, const std::vector<SceneObject *> &sceneObjectsList) const;

    void BVHProfiling(const std::vector<SceneObject *> &sceneObjectsList);

    // traversal logic
    void traceRay(Camera cam, int xstart, int xend, int ystart, int yend, int its, std::vector<SceneObject *> &sceneobjectsList, std::mutex &mutex) const;

    // bounce logic
    void sampleReflectionDirection(Ray &ray, const SceneObject &sceneObject, bool flipNormal) const;

    void sampleRefractionDirection(Ray &ray, const SceneObject &sceneObject, bool flipNormal) const;

    // tone mapping
    void toneMap(float maxLuminance, int xstart, int xend, int ystart, int yend, std::mutex &mutex);

    // multithreading logic
    std::pair<int, int> threadSegments(float start, float end, int &numThreads, std::pair<int, int>, int i);

    // cleanup
    void initialiseObjects();

    void deleteObjects();

private:
    std::vector<BVHNode *> BVHNodes;
    mutable std::vector<float> lumR, lumG, lumB; // mutable - no two threads will ever rw the same index
    mutable std::vector<float> absR, absG, absB;
    mutable std::vector<Ray *> rays;
    float maxLuminance, currentLuminance;
    mutable uint8_t *RGBBuffer;

    int resX, resY, internalResX, internalResY, iterations, numThreads, mouseX, mouseY;
    float aspectRatio, fovYRad, fovXRad, scaleX, scaleY;
    Camera &cam;
    std::pair<int, int> boundsX;
    std::pair<int, int> boundsY;

    bool running, sceneUpdated, camMoved, lockInput;
    static thread_local std::mt19937 rng; // Thread-local RNG
    mutable std::uniform_real_distribution<float> dist;

    struct BounceInfo {
        float outAngle;
        float inAngle;
        float metallic;
        float emission;
        Vector3 colour;
    };

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

#endif //RENDER_H
