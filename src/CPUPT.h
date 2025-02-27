#ifndef RENDER_H
#define RENDER_H

#include <atomic>
#include <vector>
#include <mutex>
#include <numbers>
#include <random>
#include <stack>

#include "BoundingBox.h"
#include "SceneObject.h"
#include "BVHNode.h"
#include "Camera.h"
#include "Ray.h"
#include "Window.h"
#include "Renderer.h"

class SystemManager;

class CPUPT {
public:
    struct BVHResult {
        int sceneObject;
        float close;
        float far;
    };

    CPUPT(SystemManager* systemManager);

    ~CPUPT() {
    }

    // render loop
    void renderLoop();

    // render controller
    void launchRenderThread(std::vector<SceneObject *> &sceneobjectsList);
    void joinRenderThread();

    // bvh logic
    void constructBVHST(const std::vector<SceneObject *> &sceneObjectsList);

    void constructBVHMT(const std::vector<SceneObject *> &sceneObjectsList);

    void findBestPair(const std::vector<BVHNode *> &nodes, int start, int end, std::atomic<float> &globalBestCost, int &leftIndex, int &rightIndex,
                      BVHNode *&bestLeft, BVHNode *&bestRight, std::mutex &mutex);

    void constructLinearBVH(const std::vector<SceneObject *> &sceneObjectsList);

    BVHResult searchLinearBVH(Ray &ray, const std::vector<SceneObject *> &sceneObjectsList) const;

    void BVHProfiling(const std::vector<SceneObject *> &sceneObjectsList);

    // traversal logic
    void traceRay(Camera camera, int xstart, int xend, int ystart, int yend, int its, std::mutex &mutex) const;

    // bounce logic
    Vector3 sampleSpecularDirection(Ray &ray, const SceneObject &sceneObject, bool flipNormal) const;

    Vector3 sampleDiffuseDirection(Ray &ray, const SceneObject &sceneObject, bool flipNormal) const;

    Vector3 sampleRefractionDirection(Ray &ray, const SceneObject &sceneObject, bool flipNormal) const;

    // BRDF / PDF
    float distrubtionGGX(const Vector3 &normal, const Vector3 &halfVector, float roughness) const; // Microfacet Distribution (D)

    float geometrySchlickGGX(float NdotV, float roughness) const; // Geometrey Term (G)

    float geometrySmithGGX(const Vector3 &normal, const Vector3 &viewDir, const Vector3 &lightDir, float roughness) const;

    Vector3 computeMicrofacetBRDF(float D, float F, float G, Vector3 n, Vector3 wo, Vector3 wi, Vector3 col) const { // specular
        Vector3 numerator = col * D * F * G;
        float denominator = 4.0f * fabs(n.dot(wo)) * fabs(n.dot(wi));
        return numerator / denominator;
    }

    float microfacetPDF(Vector3 &n, Vector3& wo, Vector3 &wi, float D, Vector3& h) const { // specular
        float nDotH = fabs(n.dot(h));
        float woDotH = fabs(wo.dot(h));

        if (nDotH < 1e-8f) {
            std::cout<<"nDotH < 1e-8f"<<std::endl;
        }
        if (woDotH < 1e-8f) {
            std::cout<<"nDotH < 1e-8f"<<std::endl;
        }

        return (D * nDotH) / (4.0f * woDotH);
    }

    Vector3 computeRefractionBRDF() const { // refraction
        return Vector3(1, 1, 1);
    };
    float refractionPDF() const { // refraction
        return 1;
    }

    Vector3 diffuseBRDF(Vector3 col) const { // diffuse
        return col / std::numbers::pi;
    }
    float diffusePDF(float cosTheta) const { // diffuse
        return cosTheta / std::numbers::pi;
    }

    // cosTheta is the angle between the incoming ray (rayDir) and the normal
    float fresnelSchlick(float cosTheta, float ior) const;

    // tone mapping
    void toneMap(float maxLuminance, int xstart, int xend, int ystart, int yend, std::mutex &mutex);

    // multithreading logic
    std::pair<int, int> threadSegments(float start, float end, int &numThreads, int i);

    // cleanup
    void initialiseObjects();
    void updateUpscaling();

    void deleteObjects();

    // Vector functions
    float dot(Vector3& first, Vector3& second) const {
        float dot = first.x * second.x + first.y * second.y + first.z * second.z;
        return dot;
    }

    Vector3 mix(const Vector3 &a, const Vector3 &b, float factor) const {
        return a * (1.0f - factor) + b * factor;
    }

    Vector3 normalOfHalfAngle(const Vector3 &wo, const Vector3 &wi) const {
        Vector3 h = wo + wi;
        h.normalise();
        return h;
    }


private:

    SystemManager* systemManager;

    std::vector<SceneObject *> sceneObjectsList;
    std::vector<BVHNode *> BVHNodes;
    Camera* camera;

    mutable std::vector<float> lumR, lumG, lumB; // mutable - no two threads will ever rw the same index
    mutable std::vector<float> hdrR, hdrG, hdrB;
    float maxLuminance, currentLuminance;
    mutable uint8_t *RGBBuffer;

    int resX, resY, internalResX, internalResY, iterations, numThreads, mouseX, mouseY, upScale;
    float aspectRatio;

    std::pair<int, int> boundsX;
    std::pair<int, int> boundsY;

    static thread_local std::mt19937 rng; // Thread-local RNG
    mutable std::uniform_real_distribution<float> dist;

    std::thread renderThread;

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