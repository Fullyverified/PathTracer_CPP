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
#include "Window.h"
#include "Renderer.h"

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"


class Render {
public:
    struct BVHResult {
        int sceneObject;
        float close;
        float far;
    };

    Render(Camera &cam);

    ~Render() {
        ImGui_ImplSDLRenderer2_Shutdown();
        ImGui_ImplSDL2_Shutdown();
        ImGui::DestroyContext();

        delete window;
        delete renderer;
    }

    // render loop
    void renderLoop(std::vector<SceneObject *> &sceneobjectsList);

    // game loop - input, ui, etc, calls the render loop
    void gameLoop(std::vector<SceneObject *> &sceneobjectsList);

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

    Window* window;
    Renderer* renderer;

    std::vector<BVHNode *> BVHNodes;
    mutable std::vector<float> lumR, lumG, lumB; // mutable - no two threads will ever rw the same index
    mutable std::vector<float> absR, absG, absB;
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
        float dot;
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