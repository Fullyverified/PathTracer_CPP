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

struct Reservoir {
    Vector3 candidatePosition;
    float weightSum; // Sum of weights from all candidates considered
    int sampleCount; // Number of samples seen

    float distToLight;
    float candidatePDF; // PDF of the light
    float targetPDF;
    // Light point properties
    Vector3 candidateEmission; // Selected indirect radiance
    Vector3 candidateNormal;
    Material* lightMat;
    float lightArea;
    // Hit point properties
    Vector3 rayPos;
    Vector3 n;
    Vector3 wo;
    Material* hitMat;

    Reservoir() : candidatePosition(Vector3(0, 0, 0)), candidateEmission(Vector3(0,0,0)), candidatePDF(0.0f), targetPDF(0.0f), weightSum(0.0f), sampleCount(0.0f),
    rayPos(Vector3(0.0f)), n(Vector3(0.0f)), wo(Vector3(0.0f)), distToLight(0.0f), lightMat(nullptr), hitMat(nullptr), lightArea(0.0f) {}
};

struct ReservoirGI {
    Vector3 candidatePosition;
    Vector3 candidateEmission; // Selected indirect radiance
    float PDF; // PDF of explicity sampled sample
    float weightSum; // Sum of weights from all candidates considered
    int sampleCount; // Number of samples seen

    float distToLight;

    ReservoirGI() : candidatePosition(Vector3(0, 0, 0)), candidateEmission(Vector3(0,0,0)), PDF(0.0f), weightSum(0.0f), sampleCount(0.0f), distToLight(0.0f) {}
};

struct MotionVector {
    float x, y;

    MotionVector() : x(0.0f), y(0.0f) {}
};

struct Pixel {
    int x, y;
};

class CPUPT {
public:

    CPUPT(SystemManager *systemManager, std::vector<SceneObject *> &sceneObjectsList);
    ~CPUPT() {
    }

    // render loop
    void renderLoop();

    // render controller
    void launchRenderThread();
    void joinRenderThread();

    // traversal logic
    void traceRay(Camera camera, int xstart, int xend, int ystart, int yend, int its, int currentRay, bool sky, std::mutex &mutex) const;

    // Computes resoivers
    void reservoirUpdate(Reservoir &r, Reservoir& candidate, float weight) const;

    void restirDirectLighting(Ray& ray, SceneObject* hitObject, int x, int y) const;
    // Computes direct lighting contribution using resoivers, spatiotemporaly
    void restirSpatioTemporal(int xstart, int xend, int ystart, int yend, int its, int currentRay, std::mutex &mutex) const;
    Pixel neighbourCandidate(int x, int y) const;

    // multithreading logic
    std::pair<int, int> threadSegments(float start, float end, int &numThreads, int i);

    // cleanup
    void initialiseObjects();
    void updateUpscaling();

    void deleteObjects();

    // UI Functions
    SceneObject* getClickedObject(int screenX, int screenY);
    void debugRay(int screenX, int screenY);
    void debugPixelInfo(int screenX, int screenY);


private:

    SystemManager* systemManager;
    DirectionSampler* directionSampler;
    SurfaceIntegrator* surfaceIntegrator;
    Denoiser* denoiser;
    ToneMapper* toneMapper;

    // scene objects
    std::vector<SceneObject *>& sceneObjectsList;
    std::vector<SceneObject*> emissiveObjects; // for Restir Direct Illumination
    BVHNode* rootNode;
    Camera* camera;

    // mutable - no two threads will ever write the same index
    mutable std::vector<Vector3> lum;   // pixel buffers
    mutable std::vector<Vector3> hdr;
    mutable std::vector<Reservoir> reservoirReSTIR; // ReSTIR Resoivers
    mutable std::vector<ReservoirGI> reservoirReSTIRGI; // ReSTIR GI Resoivers
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
    mutable int total = 1;

    mutable std::uniform_real_distribution<float> dist;
};

#endif //RENDER_H