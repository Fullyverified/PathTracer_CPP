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

    Vector3 sampleRefractionDirectionSingle(Ray &ray, SceneObject &sceneObject) const;
    Vector3 sampleRefractionDirectionAll(Ray &ray, SceneObject &sceneObject) const;


    // BRDF / PDF
    float distrubtionGGX(const Vector3 &normal, const Vector3 &halfVector, float roughness) const; // Microfacet Distribution (D)

    float geometrySchlickGGX(float NdotV, float roughness) const; // Geometrey Term (G)

    float geometrySmithGGX(const Vector3 &normal, const Vector3 &viewDir, const Vector3 &lightDir, float roughness) const;

    Vector3 fresnelSchlickSpecular(float cosTheta, Vector3 F0) const;

    float fresnelSchlickRefraction(float cosTheta, float IOR) const;

    Vector3 computeMicrofacetBRDF(float D, Vector3& F0, float G, Vector3& n, Vector3& wo, Vector3& wi, Vector3& col) const { // specular
        Vector3 numerator = D * F0 * G;
        float denominator = 4.0f * fabs(n.dot(wo)) * fabs(n.dot(wi));
        return numerator / denominator;
    }

    float microfacetPDF(Vector3 &n, Vector3& wo, Vector3 &wi, float D, Vector3& h) const { // specular
        float nDotH = fabs(n.dot(h));
        float woDotH = fabs(wo.dot(h));

        if (nDotH < 1e-8f || woDotH < 1e-8f) {
            std::cout<<"nDotH < 1e-8f"<<std::endl;
            std::cout<<"nDotH < 1e-8f"<<std::endl;
        }

        return (D * nDotH) / (4.0f * woDotH);
    }

    Vector3 throughputSpecularMetallic(Vector3& wo, Vector3& wi, Material& mat, Vector3& n) const {

        /*float R0 = (mat.IOR - 1.0f) / (mat.IOR + 1.0f);
        R0 = R0 * R0;
        Vector3 F0 = R0 + mat.metallic * (mat.colour - R0);*/

        Vector3 F0 = mat.colour; // place holder

        Vector3 h = normalOfHalfAngle(wo, wi);
        float cosTheta = std::abs(dot(wo, n));
        Vector3 F = fresnelSchlickSpecular(cosTheta, F0);
        float D = distrubtionGGX(n, h, mat.roughness);
        float G = geometrySmithGGX(n, wo, wi, mat.roughness);

        Vector3 brdf = computeMicrofacetBRDF(D, F, G, n, wo, wi, mat.colour);
        float pdf = microfacetPDF(n, wo, wi, D, h);

        float cosTheta_I = std::abs(n.dot(wi));;

        return brdf * cosTheta * cosTheta_I / pdf;
    }

    Vector3 throughputSpecularDiffuseRefraction(Vector3& wo, Vector3& wi, Material& mat, Vector3& n) const {

        float R0 = (mat.IOR - 1.0f) / (mat.IOR + 1.0f);
        R0 = R0 * R0;

        Vector3 h = normalOfHalfAngle(wo, wi);
        float cosTheta = std::abs(dot(wo, n));
        Vector3 F = fresnelSchlickSpecular(cosTheta, R0);
        float D = distrubtionGGX(n, h, mat.roughness);
        float G = geometrySmithGGX(n, wo, wi, mat.roughness);

        Vector3 brdf = computeMicrofacetBRDF(D, F, G, n, wo, wi, mat.colour);
        float pdf = microfacetPDF(n, wo, wi, D, h);

        float cosTheta_I = std::abs(n.dot(wi));;

        return brdf * cosTheta * cosTheta_I / pdf;
    }

    Vector3 computeRefractionBRDF(Vector3 col, float F, float n1, float n2) const { // refraction
        return (1 - F) * ((n1 / n2) * (n1 / n2)) * col;
    };
    float refractionPDF() const { // refraction
        // For a delta lobe, it’s customary to treat the PDF = 1
        // because sampling that single direction has probability 1 in that branch.
        return 1;
    }

    Vector3 throughputRefraction(Vector3& wo, Vector3& wi, const Material& mat, Vector3& n, bool internal) const {

        float cosTheta = std::abs(dot(wo, n));
        float F = fresnelSchlickRefraction(cosTheta, mat.IOR);
        float N1, N2;
        if (!internal) {
            N1 = 1.0003;
            N2 = mat.IOR;
        } else {
            N1 = mat.IOR;
            N2 = 1.0003;
        }

        Vector3 brdf = computeRefractionBRDF(mat.colour, F, N1, N2);
        float pdf = refractionPDF();

        return brdf / pdf;
    }

    Vector3 diffuseBRDF(Vector3 col) const { // diffuse
        return col / std::numbers::pi;
    }
    float diffusePDF(float cosTheta) const { // diffuse
        return cosTheta / std::numbers::pi;
    }

    Vector3 throughputDiffuse(Vector3& wo, Vector3& wi, Vector3& n, const Material& mat) const {
        float cosTheta = fabs(n.dot(wi));
        Vector3 brdf = diffuseBRDF(mat.colour);
        float pdf = diffusePDF(cosTheta);

        //return mat.colour * cosTheta;
        return brdf * cosTheta / pdf;
    }

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

    Vector3 reflect(Vector3& dir, Vector3& normal) const {
        float dotP = dot(dir, normal);
        return dir - normal * dotP * 2;
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

    bool debug;
};

#endif //RENDER_H