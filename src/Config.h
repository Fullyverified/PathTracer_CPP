#ifndef CONFIG_H
#define CONFIG_H

struct Config {
    float resX = 1200;
    float aspectX = 1;
    float aspectY = 1;
    float resY = 1200;
    float upScale = 4; // 1 is default - integer scaling - multiples of res X
    float fOV = 45;
    int raysPerPixel = 1;
    int bounceDepth = 6;
    int trisPerNode = 1;
    bool DepthOfField = false;
    float apertureRadius = 0.05f;
    float focalDistance = 15.0f;
    int threads = 1; // 0 = default
    bool denoise = false;
    float denoiseWeight = 0.75;
    float ISO = 1;
    float mouseSensitivity = 0.1f;

    void increaeISO() {
        ISO += (ISO * 0.01f);
    }

    void decreaseISO() {
        ISO -= (ISO * 0.01f);
        if (ISO < 0) {
            ISO = 0;
        }
    }

    void resetISO() {
        ISO = 1.0f;
    }

    void increaeFOV() {
        fOV += 0.1f;
        if (fOV > 180) {fOV = 180;}
    }

    void decreaeFOV() {
        fOV -= 0.1f;
        if (fOV < 0) {fOV = 0;}
    }

    void increaseAperture() {
        apertureRadius += (apertureRadius * 0.01f);
    }

    void decreaseAperture() {
        apertureRadius -= apertureRadius * 0.01f;
        if (apertureRadius < 0) {apertureRadius = 0;}
    }

    void increaseFocalDistance() {
        focalDistance += focalDistance * 0.01f;
    }

    void decreaseFocalDistance() {
        focalDistance -= focalDistance * 0.01f;
        if (focalDistance < 0) {focalDistance = 0;}
    }
};

extern Config config; // make config globally avaliable

#endif //CONFIG_H
