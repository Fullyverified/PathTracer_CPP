#ifndef CONFIG_H
#define CONFIG_H
#include <limits>

struct Config {
    float resX = 1200;
    float upScale = 6; // 1 is default - integer scaling - multiples of res X
    float aspectX = 1;
    float aspectY = 1;
    float fOV = 20;
    int raysPerPixel = 1;
    int bounceDepth = 8;
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
        fOV = fOV < 180 ? fOV += 0.1f : 180;
    }

    void decreaeFOV() {
        fOV = fOV > 0 ? fOV -= 0.1f : 0;
    }
};

extern Config config; // make config globally avaliable

#endif //CONFIG_H
