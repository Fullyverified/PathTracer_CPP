#ifndef DENOISER_H
#define DENOISER_H

#include <mutex>
#include <vector>
#include <barrier>
#include "Vector3.h"

struct DenoiseInput {
    // raw pixel data
    mutable std::vector<float> lumR;
    mutable std::vector<float> lumG;
    mutable std::vector<float> lumB;

    mutable std::vector<Vector3> normalBuffer; // normal of primary ray hit
    mutable std::vector<float> depthBuffer; // depth of primary ray hit
    mutable std::vector<Vector3> albedoBuffer; // surface colour of primary ray hit
    mutable std::vector<float> emissionBuffer; // surface colour of primary ray hit
    int numIterations; // number of denoising passes

    int resX, resY;
};

struct DenoiseOutput {
    std::vector<float> lumR;
    std::vector<float> lumG;
    std::vector<float> lumB;
};

class Denoiser {
public:

    Denoiser(int res);
    ~Denoiser();

    DenoiseOutput launchDenoiseThreads(DenoiseInput& denoiseInput, int segments);

    void A_TrousWaveletDenoising(DenoiseInput& denoiseInput, int xstart, int xend, int ystart, int yend, std::mutex& mutex, std::barrier<>& syncBarrier);

    // Vector functions
    float dot(Vector3& first, Vector3& second) const {
        float dot = first.x * second.x + first.y * second.y + first.z * second.z;
        return dot;
    }

    float distance(Vector3& first, Vector3& second) const {
        return std::sqrtf((first.x - second.x) * (first.y - second.y) * (first.z - second.z));
    }

    std::pair<int, int> threadSegments(float start, float end, int &numThreads, int step) {
        int res = end - start;
        int pixelsPerThread = res / numThreads; // number of pixels per thread
        int remainder = res % numThreads; // remaining pixels after division

        int startPixel = step * pixelsPerThread + std::min(step, remainder);
        int endPixel = (step + 1) * pixelsPerThread + std::min(step + 1, remainder) - 1;

        return std::make_pair(startPixel, endPixel);
    }

    void resize(int res) {
        lumR.resize(res, 0.0f);
        lumG.resize(res, 0.0f);
        lumB.resize(res, 0.0f);
    }

private:
    std::pair<int, int> boundsX;
    std::pair<int, int> boundsY;

    int numThreads;

    // denoised pixels
    mutable std::vector<float> lumR;
    mutable std::vector<float> lumG;
    mutable std::vector<float> lumB;
};

#endif //DENOISER_H
