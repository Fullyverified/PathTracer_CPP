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
#include "ToneMapper.h"

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
    void traceRay(Camera camera, int xstart, int xend, int ystart, int yend, int its, bool sky, std::mutex &mutex) const;

    // multithreading logic
    std::pair<int, int> threadSegments(float start, float end, int &numThreads, int i);

    // cleanup
    void initialiseObjects();
    void updateUpscaling();

    void deleteObjects();

    // UI Functions
    SceneObject* getClickedObject(int screenX, int screenY);
    void debugRay(int screenX, int screenY);


private:

    SystemManager* systemManager;
    DirectionSampler* directionSampler;
    SurfaceIntegrator* surfaceIntergrator;
    Denoiser* denoiser;
    ToneMapper* toneMapper;

    // scene objects
    std::vector<SceneObject *>& sceneObjectsList;
    BVHNode* rootNode;
    Camera* camera;

    // pixel buffers
    mutable std::vector<Vector3> lum;   // mutable - no two threads will ever rw the same index
    mutable std::vector<Vector3> hdr;
    float maxLuminance, currentLuminance;
    mutable uint8_t* RGBBuffer;

    int resX, resY, internalResX, internalResY, iterations, numThreads, mouseX, mouseY, upScale;
    float aspectRatio;

    // denoising
    mutable std::vector<Vector3> normalBuffer;
    mutable std::vector<float> depthBuffer;
    mutable std::vector<Vector3> albedoBuffer;
    mutable std::vector<float> emissionBuffer;

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