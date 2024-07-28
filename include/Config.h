//
// Created by hazza on 24/07/2024.
//

#ifndef CONFIG_H
#define CONFIG_H

struct Config {

        int resX = 3440;
        int aspectX = 4;
        int aspectY = 4;
        int fOV = 52;
        int frameTime = 100; // Milliseconds
        int raysPerPixel = 25;
        int bouncesPerRay = 5;
        bool ASCIIMode = false;
        float primaryRayStep = 0.01;
        float secondaryRayStep = 0.01;
        bool denoise = false;
        float denoiseWeight = 0.75;
        float ISO = 35; // up and down keys to modify
    };

#endif //CONFIG_H
