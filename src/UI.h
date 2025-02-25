#ifndef UI_H
#define UI_H

class UI {
public:
    static void render();

    static int accumulatedRays;
    static int numRays;
    static int numBounces;
    static bool accumulateRays;
    static float gigaRays;

    static int upscale;

    static float fOV;

    static bool depthOfField;
    static float apetureRadius;
    static float focalDistance;

    static bool camUpdate;
    static bool sceneUpdate;
    static bool upscalingUpdate;

private:

};

#endif //UI_H
