#ifndef CONFIG_H
#define CONFIG_H

struct Config {
    float resX = 1200;
    float upScale = 4; // 1 is default - integer scaling - multiples of res X
    float aspectX = 1;
    float aspectY = 1;
    float fOV = 53;
    int raysPerPixel = 1;
    int bounceDepth = 7;
    float apertureRadius = 0.001f;
    float focalDistance = 2.0f;;
    int threads = 0; // 0 = default
    bool denoise = false;
    float denoiseWeight = 0.75;
    float ISO = 1; // up and down keys to modify
    float mouseSensitivity = 0.1f;

    void increaeISO() {
        ISO += (ISO * 0.01f);
    }

    void decreaseISO() {
        ISO = ISO > 0.05f ? ISO - (ISO * 0.01f) : 0.05f;
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

    void increaseApeture() {
        apertureRadius += (apertureRadius * 0.01f);
    }

    void decreaseApeture() {
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
