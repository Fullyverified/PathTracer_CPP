#include "ToneMapper.h"
#include "config.h"

void ToneMapper::launchToneMappingThreads(std::vector<Vector3> &colour, uint8_t *RGBBuffer, float maxLuminance, int segments, int resx, int resy, int internalResX, int internalResY, int upScale, int numThreads) {

    // Compute Max luminance
    float currentLuminance = 0;
    for (int i = 0; i < internalResX * internalResY; i++) {
        // determine brightest amplitude in scene
        currentLuminance = 0.2126f * colour[i].x + 0.7152f * colour[i].y + 0.0722f * colour[i].z;
        maxLuminance = currentLuminance > maxLuminance ? currentLuminance : maxLuminance;
    }
    std::mutex mutex;

    // Calculate number of screen segments
    int tileSize = config.tileSize;
    int segmentsX = (internalResX + tileSize - 1) / tileSize; // Ceiling division
    int segmentsY = (internalResY + tileSize - 1) / tileSize;
    int totalSegments = segmentsX * segmentsY;
    std::atomic<int> nextSegment(0);

    // Create a worker for each segment
    auto workerToneMap = [this, &nextSegment, totalSegments, segmentsX, tileSize, &mutex, &colour, &RGBBuffer, internalResX, internalResY, resx, upScale, maxLuminance]() {
        while (true) {
            int segmentIndex = nextSegment.fetch_add(1);
            if (segmentIndex >= totalSegments) {
                break;
            }
            // Convert segment index to tile coordinates
            int tileY = segmentIndex / segmentsX;
            int tileX = segmentIndex % segmentsX;
            int startX = tileX * tileSize;
            int endX = std::min(startX + tileSize - 1, internalResX - 1);
            int startY = tileY * tileSize;
            int endY = std::min(startY + tileSize - 1, internalResY - 1);
            extended_Reinhard(colour, RGBBuffer,
                                              startX, endX, startY,
                                              endY, internalResX, resx, upScale, maxLuminance, std::ref(mutex));
        }
    };

    // Launch worker threads
    std::vector<std::thread> threadsTM;
    for (int i = 0; i < numThreads; ++i) {
        threadsTM.emplace_back(workerToneMap);
    }
    // Block until all threads are finished

    for (std::thread &thread: threadsTM) {
        thread.join();
    }
    threadsTM.clear();
}

void ToneMapper::extended_Reinhard(std::vector<Vector3> &HDR, uint8_t *RGBBuffer, int xstart, int xend, int ystart, int yend, int internalResX, int resX, int upScale, float maxLuminance, std::mutex &mutex) {

    for (int x = xstart; x <= xend; x++) {
        for (int y = ystart; y <= yend; y++) {

            Vector3 colour = HDR[y * internalResX + x];

            float luminance = 0.2126f * colour.x + 0.7152f * colour.y + 0.0722f * colour.z;

            if (luminance > 0) {

                // Extended Reinhard Tone Mapping - returns value [0, 1]
                float mappedLuminance = (luminance * (1 + (luminance / (maxLuminance * maxLuminance)))) / (1 + luminance);


                colour = colour / luminance * mappedLuminance;

                // Apply gamma correction
                float gamma = 2.2f;
                float invGamma = 1.0f / gamma;

                colour.x = pow(colour.x, invGamma);
                colour.y = pow(colour.y, invGamma);
                colour.z = pow(colour.z, invGamma);

                colour = colour * 255;

                colour.x = std::min(colour.x, 255.0f);
                colour.y = std::min(colour.y, 255.0f);
                colour.z = std::min(colour.z, 255.0f);
            }

            // upscale and store in RGB buffer considering aspect ratio
            for (int i = 0; i < upScale; i++) {
                for (int j = 0; j < upScale; j++) {
                    int outX = static_cast<int>(x * upScale + i);
                    int outY = static_cast<int>(y * upScale + j);
                    int offset = (outY * resX + outX) * 3;

                    RGBBuffer[offset] = colour.x;
                    RGBBuffer[offset + 1] = colour.y;
                    RGBBuffer[offset + 2] = colour.z;
                }
            }
        }
    }

}
