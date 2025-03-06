#ifndef RENDER_H
#define RENDER_H

#include <vector>
#include <mutex>
#include <random>

#include "SceneObject.h"
#include "Camera.h"
#include "Ray.h"

#include "Renderer.h"
#include "DirectionSampler.h"
#include "SurfaceIntegrator.h"

#include "Denoiser.h"

class SystemManager;

class CPUPT {
public:

    CPUPT(SystemManager* systemManager, std::vector<SceneObject *>& sceneObjectsList);
    ~CPUPT() {
    }

    // render loop
    void renderLoop();

    // render controller
    void launchRenderThread();
    void joinRenderThread();

    // traversal logic
    void traceRay(Camera camera, int xstart, int xend, int ystart, int yend, int its, bool sky, DenoiseInput& denoiseInput, std::mutex &mutex) const;

    // tone mapping
    void toneMap(float maxLuminance, int xstart, int xend, int ystart, int yend, std::mutex &mutex);

    // multithreading logic
    std::pair<int, int> threadSegments(float start, float end, int &numThreads, int i);

    // cleanup
    void initialiseObjects(DenoiseInput& denoiseInput);
    void updateUpscaling(DenoiseInput& denoiseInput);

    void deleteObjects();

    // UI Functions
    SceneObject* getClickedObject(int screenX, int screenY);
    void debugRay(int screenX, int screenY);


private:

    SystemManager* systemManager;
    DirectionSampler* directionSampler;
    SurfaceIntegrator* surfaceIntergrator;
    Denoiser* denoiser;

    // scene objects
    std::vector<SceneObject *>& sceneObjectsList;
    BVHNode* rootNode;
    Camera* camera;

    // pixel buffers
    mutable std::vector<float> lumR, lumG, lumB; // mutable - no two threads will ever rw the same index
    mutable std::vector<float> hdrR, hdrG, hdrB;
    float maxLuminance, currentLuminance;
    mutable uint8_t *RGBBuffer;

    int resX, resY, internalResX, internalResY, iterations, numThreads, mouseX, mouseY, upScale;
    float aspectRatio;

    // multithreading
    std::pair<int, int> boundsX;
    std::pair<int, int> boundsY;

    std::pair<int, int> boundsXThread;
    std::pair<int, int> boundsYThread;

    std::thread renderThread;

    static thread_local std::mt19937 rng; // Thread-local RNG

    bool debug;
};

#endif //RENDER_H