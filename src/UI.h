#ifndef UI_H
#define UI_H

#include "MaterialManager.h"
#include "SceneObjectManager.h"
#include "SceneObject.h"

class UI {
public:
    static bool isWindowHovered;

    static void renderSettings();
    static void materialEditor();
    static void sceneEditor();

    static int RaysPerSecond;
    static float pathTracingTime;
    static float denoisingTime;
    static float toneMappingTime;
    static float frameTime;
    static bool sky;

    // MIS
    static int accumulatedRays;
    static int numRays;
    static int minBounces;
    static int maxBounces;
    static bool accumulateRays;
    static bool BSDF;
    static bool NEE;
    static int NEEsamples;

    // ReSTIR
    static bool ReSTIR;
    static bool unbiased;
    static int candidateSamples;
    static int spatialSamplesK;
    static int temporalSampling;

    static bool ReSTIRGI;

    // Denoising
    static bool denoise;
    static int denoiseIterations;

    static int numThreads;

    static int upscale;

    // Camera
    static float fOV;
    static float exposure;

    static bool depthOfField;
    static float apetureRadius;
    static float focalDistance;

    static int resX;
    static int resY;

    static Vector3 camPos;
    static Vector3 camDir;

    static bool camUpdate;
    static bool sceneUpdate;
    static bool upscalingUpdate;
    static bool resUpdate;
    static bool resizeBuffer;

    // Material Editing
    static Vector3 colour;
    static float roughness;
    static float metallic;
    static float IOR;
    static float transmission;
    static float emission;

    static MaterialManager* materialManager;
    static std::string materialKey;
    static std::string newMatName;

    // Scene Editing
    static SceneObject* selectedObject;

    static SceneObjectManager* sceneObjectManager;
    static int primativeSelection;
    static int meshSelection;

private:

};

#endif //UI_H
