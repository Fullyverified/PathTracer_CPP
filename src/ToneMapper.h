#ifndef TONEMAPPER_H
#define TONEMAPPER_H

#include <vector>
#include <mutex>
#include <future>

#include "Vector3.h"

class ToneMapper {
public:

    ToneMapper() {

    }
    ~ToneMapper() {

    }

    void launchToneMappingThreads(std::vector<Vector3>& colour, uint8_t* RGBBuffer, float maxLuminance, int segments, int resx, int resy, int internalResX, int internalResY, int upScale);

    void extended_Reinhard(std::vector<Vector3>& HDR, uint8_t *RGBBuffer, int xstart, int xend, int ystart, int yend, int internalResX, int resX, int upScale, float maxLuminance, std::mutex& mutex);

    std::pair<int, int> threadSegments(float start, float end, int &numThreads, int step) {
        int res = end - start;
        int pixelsPerThread = res / numThreads; // number of pixels per thread
        int remainder = res % numThreads; // remaining pixels after division

        int startPixel = step * pixelsPerThread + std::min(step, remainder);
        int endPixel = (step + 1) * pixelsPerThread + std::min(step + 1, remainder) - 1;

        return std::make_pair(startPixel, endPixel);
    }
private:
    std::pair<int, int> boundsX;
    std::pair<int, int> boundsY;
};


#endif //TONEMAPPER_H
