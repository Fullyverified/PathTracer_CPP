#include "ToneMapper.h"


void ToneMapper::launchToneMappingThreads(std::vector<Vector3> &colour, uint8_t *RGBBuffer, float maxLuminance, int segments, int resx, int resy, int internalResX, int internalResY, int upScale) {

    // Compute Max luminance
    float currentLuminance = 0;
    for (int i = 0; i < internalResX * internalResY; i++) {
        // determine brightest amplitude in scene
        currentLuminance = 0.2126f * colour[i].x + 0.7152f * colour[i].y + 0.0722f * colour[i].z;
        maxLuminance = currentLuminance > maxLuminance ? currentLuminance : maxLuminance;
    }

    // Tone Map

    std::vector<std::future<void>> threads;
    std::mutex mutex;

    for (int j = 0; j < segments; j++) {
        for (int i = 0; i < segments; i++) {
            boundsX = threadSegments(0, internalResX, segments, i);
            boundsY = threadSegments(0, internalResY, segments, j);
            threads.emplace_back(std::async(std::launch::async, &ToneMapper::extended_Reinhard, this, std::ref(colour), std::ref(RGBBuffer),
                                              boundsX.first, boundsX.second, boundsY.first,
                                              boundsY.second, internalResX, resx, upScale, maxLuminance, std::ref(mutex)));
        }
    }
    for (std::future<void> &thread: threads) {
        thread.get(); // Blocks until the thread completes its task
    }
    threads.clear();

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
