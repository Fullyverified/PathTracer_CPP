//
// Created by hazza on 24/07/2024.
//

#ifndef CONFIG_H
#define CONFIG_H

struct Config {

        int resX = 800;
        int aspectX = 1;
        int aspectY = 1;
        int fOV = 52;
        int frameTime = 100; // Milliseconds
        int raysPerPixel = 100;
        int bounceDepth = 1;
        int rayPerSegment = 10;
        int memory = 4.5; // gigabytes
        bool ASCIIMode = false;
        float primaryRayStep = 0.01;
        float secondaryRayStep = 0.01;
        bool denoise = false;
        float denoiseWeight = 0.75;
        float ISO = 35; // up and down keys to modify
    };

extern Config config; // make config globally avaliable

#endif //CONFIG_H
