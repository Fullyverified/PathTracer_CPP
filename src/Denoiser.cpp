#include "Denoiser.h"

#include <future>

#include "UI.h"

Denoiser::Denoiser() {
}

Denoiser::~Denoiser() {

}

void Denoiser::launchDenoiseThreads(std::vector<Vector3>& colour, std::vector<Vector3>& normalBuffer, std::vector<float>& depthBuffer, std::vector<Vector3>& albedoBuffer, std::vector<float>& emissionBuffer, int numIterations, int resX, int resY, int segments) {

    std::vector<std::future<void>> threads;
    std::mutex mutex;

    numThreads = segments * segments;
    std::barrier syncBarrier(numThreads);

    for (int j = 0; j < segments; j++) {
        for (int i = 0; i < segments; i++) {
            boundsX = threadSegments(0, resX, segments, i);
            boundsY = threadSegments(0, resY, segments, j);
            threads.emplace_back(std::async(std::launch::async, &Denoiser::A_TrousWaveletDenoising, this, std::ref(colour), std::ref(normalBuffer), std::ref(depthBuffer), std::ref(albedoBuffer), std::ref(emissionBuffer), numIterations, resX, resY,
                                              boundsX.first, boundsX.second, boundsY.first,
                                              boundsY.second, std::ref(mutex), std::ref(syncBarrier)));
        }
    }
    for (std::future<void> &thread: threads) {
        thread.get(); // Blocks until the thread completes its task
    }
    threads.clear();
}

void Denoiser::A_TrousWaveletDenoising(std::vector<Vector3>& colour, std::vector<Vector3>& normalBuffer, std::vector<float>& depthBuffer, std::vector<Vector3>& albedoBuffer, std::vector<float>& emissionBuffer, int numIterations, int resX, int resY, int xstart, int xend, int ystart, int yend, std::mutex& mutex, std::barrier<>& syncBarrier) {


    // A-Trous kernel - pixel weights
    float kernel[5][5] = {
        {0, 0, 1, 0, 0},
        {0, 1, 2, 1, 0},
        {1, 2, 4, 2, 1},
        {0, 1, 2, 1, 0},
        {0, 0, 1, 0, 0}
    };

    int stepSize = 1;
    for (int iteration = 1; iteration <= numIterations; iteration++) {
        // Process a block of pixels
        for (int x = xstart; x <= xend; x++) {
            for (int y = ystart; y <= yend; y++) {
                float weightSum = 0.0f;
                Vector3 colourSum(0, 0, 0);

                // Precompute repeated indexes
                Vector3 pixelNormal = normalBuffer[y * resX + x];
                float pixelDepth = depthBuffer[y * resX + x];
                Vector3 pixelAlbedo = albedoBuffer[y * resX + x];
                float pixelEmission = emissionBuffer[y * resX + x];

                // Iterate over the kernel neighborhood
                for (int kx = -2; kx <= 2; kx++) {
                    for (int ky = -2; ky <= 2; ky++) {
                        int sampleX = x + kx * stepSize;
                        int sampleY = y + ky * stepSize;

                        if (sampleX < 0 || sampleX >= resX || sampleY < 0 || sampleY >= resY || depthBuffer[sampleY * resX + sampleX] == -1) {
                            continue;
                        }

                        float kernelWeight = kernel[ky + 2][kx + 2];

                        // Edge-awareness evaluation
                        float normalSimilarity = Vector3::dot(pixelNormal, normalBuffer[sampleY * resX + sampleX]);
                        float albedoSimilarity = Vector3::distance(pixelAlbedo, albedoBuffer[sampleY * resX + sampleX]);
                        float depthDifference = std::fabs(pixelDepth - depthBuffer[sampleY * resX + sampleX]);
                        //float emissionSimilarity = pixelEmission > emissionBuffer[sampleY * resX + sampleX] ? pixelEmission / emissionBuffer[sampleY * resX + sampleX] : emissionBuffer[sampleY * resX + sampleX] / pixelEmission;

                        float normalWeight = std::exp(-std::max(0.0f, 1.0f - normalSimilarity) / 0.05f);
                        float albedoWeight = std::exp(-albedoSimilarity * albedoSimilarity / (0.05f * 0.05f));
                        float depthWeight = std::exp(-depthDifference * depthDifference / (0.05f * 0.05f));
                        float intensityWeight = std::exp(-Vector3::distance(pixelAlbedo, albedoBuffer[sampleY * resX + sampleX]) / 0.5f);
                        //float emissionWeight = (emissionSimilarity < 0.9f) ? 0.2f : 1.0f;

                        float weight = kernelWeight * normalWeight * albedoWeight * depthWeight * intensityWeight;

                        weightSum += weight;
                        colourSum += colour[sampleY * resX + sampleX] * weight;
                    }
                }

                {
                    if (weightSum > 0.0f || depthBuffer[y * resX + x] != -1) {
                        tmpResult[y * resX + x] = colourSum / weightSum;
                    } else {
                        tmpResult[y * resX + x] = colour[y * resX + x];
                    }
                }
            }
        }
        // Wait for all threads to finish this iteration before proceding
        syncBarrier.arrive_and_wait();
        stepSize *= 2;

        // Update original colour bugger
        for (int x = xstart; x <= xend; x++) {
            for (int y = ystart; y <= yend; y++) {
                colour[y * resX + x] = tmpResult[y * resX + x];
            }
        }

    }
}