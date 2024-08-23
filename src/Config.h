#ifndef CONFIG_H
#define CONFIG_H

struct Config {
    int resX = 800;
    int ScaleFactor = 8; // 1 is default - integer scaling: 1, 2, 4, 8
    int aspectX = 1;
    int aspectY = 1;
    int fOV = 52;
    int raysPerPixel = 20;
    int bounceDepth = 8;
    int threads = 0; // 0 = default
    bool denoise = false;
    float denoiseWeight = 0.75;
    float ISO = 0.2; // up and down keys to modify

    void increaeISO() {
        ISO = ISO - (ISO * 0.10f);
    }

    void decreaeISO() {
        ISO = ISO + (ISO * 0.10f);
    }
};

extern Config config; // make config globally avaliable

#endif //CONFIG_H
