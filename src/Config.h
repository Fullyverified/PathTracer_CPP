#ifndef CONFIG_H
#define CONFIG_H

struct Config {
    int resX = 800;
    int upScale = 1; // 1 is default - integer scaling: 1, 2, 4, 8
    int aspectX = 1;
    int aspectY = 1;
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
