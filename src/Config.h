#ifndef CONFIG_H
#define CONFIG_H

struct Config {

        int resX = 1300;
        int aspectX = 1;
        int aspectY = 1;
        int fOV = 53;
        int frameTime = 100; // Milliseconds
        int raysPerPixel = 1000;
        int bounceDepth = 5;
        bool ASCIIMode = false;
        float primaryRayStep = 0.01;
        float secondaryRayStep = 0.01;
        bool denoise = false;
        float denoiseWeight = 0.75;
        float ISO = 0.2251650845186529; // up and down keys to modify
    };

extern Config config; // make config globally avaliable

#endif //CONFIG_H
