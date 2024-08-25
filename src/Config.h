#ifndef CONFIG_H
#define CONFIG_H

struct Config {
    float resX = 800;
    float upScale = 4; // 1 is default - integer scaling - multiples of res X
    float aspectX = 1;
    float aspectY = 1;
    float fOV = 54;
    int raysPerPixel = 1;
    int bounceDepth = 5;
    int threads = 0; // 0 = default
    bool denoise = false;
    float denoiseWeight = 0.75;
    float ISO = 1; // up and down keys to modify
    float mouseSensitivity = 0.1f;

    void increaeISO() {
        ISO = ISO - (ISO * 0.01f);
    }

    void decreaeISO() {
        ISO = ISO + (ISO * 0.01f);
    }

    void resetISO() {
        ISO = 1.0f;
    }

    void increaeFOV() {
        fOV = fOV + (0.1f);
    }

    void decreaeFOV() {
        fOV = fOV - (0.1f);
    }
};

extern Config config; // make config globally avaliable

#endif //CONFIG_H
