#ifndef UI_H
#define UI_H
#include "MaterialManager.h"

class UI {
public:
    static void renderSettings();
    static void materialEditor();
    static void sceneEditor();

    static int RaysPerSecond;
    static float pathTracingTime;
    static float toneMappingTime;
    static float frameTime;
    static int accumulatedRays;
    static int numRays;
    static int numBounces;
    static bool accumulateRays;

    static int numThreads;

    static int upscale;

    static float fOV;
    static float exposure;

    static bool depthOfField;
    static float apetureRadius;
    static float focalDistance;

    static int resX;
    static int resY;

    static bool camUpdate;
    static bool sceneUpdate;
    static bool upscalingUpdate;
    static bool resUpdate;
    static bool resizeBuffer;

    static Vector3 colour;
    static float roughness;
    static float metallic;
    static float IOR;
    static float transmission;
    static float emission;

    static MaterialManager* materialManager;
    static std::string materialKey;
    static std::string newMatName;

private:

};

#endif //UI_H
