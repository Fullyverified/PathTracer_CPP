#ifndef DENOISER_H
#define DENOISER_H

#include <mutex>
#include <vector>
#include <barrier>
#include "Vector3.h"

class Denoiser {
public:

    Denoiser();
    ~Denoiser();

    void launchDenoiseThreads(std::vector<Vector3>& colour, std::vector<Vector3>& normalBuffer, std::vector<float>& depthBuffer, std::vector<Vector3>& albedoBuffer, std::vector<float>& emissionBuffer, int numIterations, int resX, int resY, int segments);

    void A_TrousWaveletDenoising(std::vector<Vector3>& colour, std::vector<Vector3>& normalBuffer, std::vector<float>& depthBuffer, std::vector<Vector3>& albedoBuffer, std::vector<float>& emissionBuffer, int numIterations, int resX, int resY, int xstart, int xend, int ystart, int yend, std::mutex& mutex, std::barrier<>& syncBarrier);

    std::pair<int, int> threadSegments(float start, float end, int &numThreads, int step) {
        int res = end - start;
        int pixelsPerThread = res / numThreads; // number of pixels per thread
        int remainder = res % numThreads; // remaining pixels after division

        int startPixel = step * pixelsPerThread + std::min(step, remainder);
        int endPixel = (step + 1) * pixelsPerThread + std::min(step + 1, remainder) - 1;

        return std::make_pair(startPixel, endPixel);
    }

    void resize(int res) {
        tmpResult.resize(res, 0.0f);
    }

private:
    std::pair<int, int> boundsX;
    std::pair<int, int> boundsY;

    int numThreads;

    mutable std::vector<Vector3> tmpResult;
};

#endif //DENOISER_H
