#ifndef UI_H
#define UI_H

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

private:

};

#endif //UI_H
