#include "Denoiser.h"

#include <future>

#include "UI.h"

Denoiser::Denoiser(int res) {
    resize(res);
}

Denoiser::~Denoiser() {

}

void Denoiser::launchDenoiseThreads(DenoiseInput &denoiseInput, int segments) {

    std::vector<std::future<void>> threads;
    std::mutex mutex;

    numThreads = segments * segments;
    std::barrier syncBarrier(numThreads);

    for (int j = 0; j < segments; j++) {
        for (int i = 0; i < segments; i++) {
            boundsX = threadSegments(0, denoiseInput.resX, segments, i);
            boundsY = threadSegments(0, denoiseInput.resY, segments, j);
            threads.emplace_back(std::async(std::launch::async, &Denoiser::A_TrousWaveletDenoising, this, std::ref(denoiseInput),
                                              boundsX.first, boundsX.second, boundsY.first,
                                              boundsY.second, std::ref(mutex), std::ref(syncBarrier)));
        }
    }
    for (std::future<void> &thread: threads) {
        thread.get(); // Blocks until the thread completes its task
    }
    threads.clear();

}

void Denoiser::A_TrousWaveletDenoising(DenoiseInput& denoiseInput, int xstart, int xend, int ystart, int yend, std::mutex& mutex, std::barrier<>& syncBarrier) {
    int resX = denoiseInput.resX;
    int resY = denoiseInput.resY;
    int numIterations = denoiseInput.numIterations;

    std::vector<Vector3>& normalBuffer = denoiseInput.normalBuffer;
    std::vector<float>& depthBuffer = denoiseInput.depthBuffer;
    std::vector<Vector3>& albedoBuffer = denoiseInput.albedoBuffer;
    std::vector<float>& emissionBuffer = denoiseInput.emissionBuffer;



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
                        float normalSimilarity = dot(pixelNormal, normalBuffer[sampleY * resX + sampleX]);
                        float depthDifference = std::fabs(pixelDepth - depthBuffer[sampleY * resX + sampleX]);
                        float albedoSimilarity = distance(pixelAlbedo, albedoBuffer[sampleY * resX + sampleX]);
                        float emissionSimilarity = pixelEmission > emissionBuffer[sampleY * resX + sampleX] ? pixelEmission / emissionBuffer[sampleY * resX + sampleX] : emissionBuffer[sampleY * resX + sampleX] / pixelEmission;

                        float normalWeight = (normalSimilarity > 0.90f) ? 1.0f : 0.4f;
                        float depthWeight = (depthDifference < 0.04f) ? 1.0f : 0.4f;
                        float albedoWeight = (albedoSimilarity < 0.1f) ? 1.0f : 0.4f;
                        float emissionWeight = (emissionSimilarity < 0.9f) ? 0.2f : 1.0f;

                        float weight = kernelWeight * normalWeight * depthWeight * albedoWeight * emissionWeight;

                        //colourSum.x += denoiseInput.lumR[sampleY * resX + sampleX] * weight;
                        //colourSum.y += denoiseInput.lumG[sampleY * resX + sampleX] * weight;
                        //colourSum.z += denoiseInput.lumB[sampleY * resX + sampleX] * weight;
                        weightSum += weight;
                    }
                }

                {
                    std::lock_guard<std::mutex> lock(mutex);
                    if (weightSum > 0.0f || depthBuffer[y * resX + x] != -1) {
                        //lumR[y * resX + x] = colourSum.x / weightSum;
                        //lumG[y * resX + x] = colourSum.y / weightSum;
                        //lumB[y * resX + x] = colourSum.z / weightSum;
                    } else {
                        //lumR[y * resX + x] = lumR[y * resX + x];
                        //lumG[y * resX + x] = lumG[y * resX + x];
                        //lumB[y * resX + x] = lumB[y * resX + x];
                    }
                }
            }
        }
        // Wait for all threads to finish this iteration before proceding
        syncBarrier.arrive_and_wait();
        stepSize *= 2;
    }
}