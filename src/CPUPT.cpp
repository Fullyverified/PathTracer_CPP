#include "CPUPT.h"

#include "imgui.h"

#include <algorithm>
#include <chrono>
#include <future>
#include <iostream>
#include <thread>
#include <mutex>
#include <stack>
#include <vector>

#include "Config.h"

#include "Ray.h"
#include "BVHNode.h"
#include "BVH.h"
#include "DirectionSampler.h"
#include "UI.h"

#include "SystemManager.h"

thread_local std::mt19937 CPUPT::rng(std::random_device{}());

CPUPT::CPUPT(SystemManager *systemManager, std::vector<SceneObject *>& sceneObjectsList) : systemManager(systemManager), sceneObjectsList(sceneObjectsList), iterations(0), numThreads(0) {
    directionSampler = new DirectionSampler();
    surfaceIntergrator = new SurfaceIntegrator();

    camera = systemManager->getSceneObjectManager()->getCamera();
}

void CPUPT::launchRenderThread() {
    this->sceneObjectsList = sceneObjectsList;

    // launch a thread the handles the render loop
    renderThread = std::thread(&CPUPT::renderLoop, this);
}

void CPUPT::joinRenderThread() {
    if (renderThread.joinable()) {
        renderThread.join();
    }
}

void CPUPT::renderLoop() {
    // initialise objects
    std::mutex mutex;
    std::vector<std::future<void> > threads;
    std::vector<std::future<void >> threadsTM;

    // Initialise Objects
    std::cout << "Constructing Objects" << std::endl;
    initialiseObjects();
    std::cout << "Avaliable Threads: " << std::thread::hardware_concurrency() << std::endl;

    std::cout << "Constructing BVH" << std::endl;
    BVH bvh = BVH();
    bvh.constructBVHST(sceneObjectsList);
    rootNode = bvh.getBVHNodes()[0];

    // render loop code
    while (systemManager->getIsRunning()) {
        auto frameStartTime = std::chrono::high_resolution_clock::now();

        numThreads = config.threads > 0 ? config.threads : std::thread::hardware_concurrency();
        int segments = std::round(std::sqrt(numThreads));

        if (UI::camUpdate || camera->getCamMoved() || !UI::accumulateRays) {
            UI::camUpdate = false;
            camera->getCamMoved() = false;

            config.fOV = UI::fOV;
            config.exposure = UI::exposure;
            camera->reInitilize(); // fov changes

            if (UI::sceneUpdate) {
                UI::sceneUpdate = false;
                bvh.constructBVHST(sceneObjectsList);
                rootNode = bvh.getBVHNodes()[0];
            }

            if (UI::resUpdate) {
                initialiseObjects(); // change buffer size
                UI::resUpdate = false;
                UI::resizeBuffer = true;
            }

            // reset colour buffer
            for (int i = 0; i < internalResX * internalResY; i++) {
                lumR[i] = 0.0f;
                lumG[i] = 0.0f;
                lumB[i] = 0.0f;
                hdrR[i] = 0.0f;
                hdrG[i] = 0.0f;
                hdrB[i] = 0.0f;
            }
            iterations = 0;
            UI::accumulatedRays = iterations * config.raysPerPixel;
        }

        if (UI::upscalingUpdate) {
            UI::upscalingUpdate = false;
            config.upScale = UI::upscale;
            updateUpscaling();
        }

        Camera cameraCopy = *camera; // dereference camera and copy

        auto startTimeRays = std::chrono::high_resolution_clock::now();

        // Calculate number of screen segments
        int tileSize = config.tileSize;
        int segmentsX = (internalResX + tileSize - 1) / tileSize; // Ceiling division
        int segmentsY = (internalResY + tileSize - 1) / tileSize;
        int totalSegments = segmentsX * segmentsY;
        std::atomic<int> nextSegment(0);

        // Create a worker for each segment
        auto worker = [this, &nextSegment, totalSegments, segmentsX, tileSize, &cameraCopy, &mutex]() {
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
                traceRay(cameraCopy, startX, endX, startY, endY, iterations, mutex);
            }
        };

        // Launch worker threads
        std::vector<std::thread> threadsPT;
        for (int i = 0; i < numThreads; ++i) {
            threadsPT.emplace_back(worker);
        }
        // Block until all threads are finished
        for (auto& t : threadsPT) {
            t.join();
        }
        threadsPT.clear();

        auto durationTimeRays = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeRays);
        UI::pathTracingTime = std::chrono::duration<float>(durationTimeRays).count() * 1000;
        //-----------------------

        // tone mapping
        maxLuminance = 0;
        currentLuminance = 0;
        for (int i = 0; i < internalResX * internalResY; i++) {
            // determine brightest amplitude in scene
            currentLuminance = 0.2126f * lumR[i] + 0.7152f * lumG[i] + 0.0722f * lumB[i];
            maxLuminance = currentLuminance > maxLuminance ? currentLuminance : maxLuminance;
        }
        maxLuminance *= config.exposure;

        auto startTimeTM = std::chrono::high_resolution_clock::now();
        for (int j = 0; j < segments; j++) {
            for (int i = 0; i < segments; i++) {
                boundsX = threadSegments(0, internalResX, segments, i);
                boundsY = threadSegments(0, internalResY, segments, j);
                threadsTM.emplace_back(std::async(std::launch::async, &CPUPT::toneMap, this, maxLuminance,
                                                boundsX.first, boundsX.second, boundsY.first,
                                                boundsY.second, std::ref(mutex)));
            }
        }
        for (std::future<void> &thread : threadsTM) {
            thread.get(); // Blocks until the thread completes its task
        }
        threadsTM.clear();

        systemManager->updateRGBBuffer(RGBBuffer); // push latest screen buffer

        auto durationTimeTM = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeTM);
        auto finalFrameTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - frameStartTime);
        UI::toneMappingTime = std::chrono::duration<float>(durationTimeTM).count() * 1000;
        UI::frameTime = std::chrono::duration<float>(finalFrameTime).count() * 1000;

        iterations++;
        UI::accumulatedRays = iterations * config.raysPerPixel;

        float frameTimeSec = std::chrono::duration<float>(durationTimeRays).count();
        UI::RaysPerSecond = (config.raysPerPixel * internalResX * internalResY) / (frameTimeSec);
        //-----------------------
    }
}

void CPUPT::traceRay(Camera camera, int xstart, int xend, int ystart, int yend, int its, std::mutex &mutex) const {
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);

    for (int y = ystart; y <= yend; y++) {
        for (int x = xstart; x <= xend; x++) {
            for (int currentRay = 1; currentRay <= config.raysPerPixel; currentRay++) {
                // -------------------------------------------------------------
                // Primary (camera) Ray setup
                // -------------------------------------------------------------
                Ray ray;
                ray.reset();
                // Camera (primary) ray origin:
                ray.getOrigin().set(camera.getPos());
                ray.getPos().set(camera.getPos());

                // Jitter the pixel position for MSAA
                float jitterX = (static_cast<float>(rand()) / RAND_MAX - 0.5f) / internalResX;
                float jitterY = (static_cast<float>(rand()) / RAND_MAX - 0.5f) / internalResY;
                Vector3 pixelPosPlane(((((x + 0.5f + jitterX) / internalResX) * 2) - 1) * aspectRatio,
                                      1 - (((y + 0.5f + jitterY) / internalResY) * 2), 0);
                Vector3 pixelPosScene(pixelPosPlane.getX() * camera.getPlaneWidth() / 2,
                                      pixelPosPlane.getY() * camera.getPlaneHeight() / 2, 0);

                // Point ray according to pixel position (ray starts from camera origin)
                ray.getDir().set(camera.getDir() + camera.getRight() * pixelPosScene.getX() + camera.getUp() * pixelPosScene.getY());
                ray.getDir().normalise();

                // Depth of Field
                if (config.DepthOfField) {
                    Vector3 focalPoint = camera.getPos() + ray.getDir() * config.focalDistance;
                    float lensU = ((static_cast<float>(rand()) / RAND_MAX) - 0.5f) * 2.0f * config.apertureRadius;
                    float lensV = ((static_cast<float>(rand()) / RAND_MAX) - 0.5f) * 2.0f * config.apertureRadius;
                    Vector3 lensOffset = camera.getRight() * lensU + camera.getUp() * lensV;
                    ray.getOrigin().set(camera.getPos() + lensOffset);
                    ray.getPos().set(camera.getPos() + lensOffset);
                    ray.getDir().set(focalPoint - ray.getOrigin());
                    ray.getDir().normalise();
                }

                // -------------------------------------------------------------
                // Path Tracing Loop
                // -------------------------------------------------------------
                Vector3 finalColour(0.0f, 0.0f, 0.0f);
                Vector3 throughput(1.0f, 1.0f, 1.0f); // Running throughput
                ray.setInternal(false);

                for (int currentBounce = 0; currentBounce <= config.bounceDepth; currentBounce++) {

                    // Intersect scene using BVH
                    BVHNode::BVHResult leafNode = rootNode->searchBVHTreeScene(ray);
                    if (leafNode.node == nullptr || !leafNode.node->getLeaf()) {
                        break;
                    }

                    // Move the ray to the exact hit point
                    SceneObject *hitObject = leafNode.node->getSceneObject();
                    if (!ray.getInternal()) {
                        // outside of an object - march to entry
                        ray.march(leafNode.close);
                    } else {
                        // inside of an object - march to far side
                        ray.march(leafNode.far);
                    }

                    hitObject->getNormal(ray); // sets ray.getNormal()
                    ray.getOrigin().set(ray.getPos()); // Set new ray origin
                    ray.setHitObject(hitObject);

                    // Grab the material
                    Material* mat = hitObject->getMaterial();

                    // 1) Sample new direction: reflection or refraction and compute BRDF and PDF
                    float randomSample = dist(rng);
                    float p_specular = mat->metallic;
                    float p_transmission = mat->transmission * (1.0f - mat->metallic);
                    float p_diffuse = 1.0f - (p_specular + p_transmission);
                    // probabilites should add up to 1

                    Vector3 wo = ray.getDir() * -1; // outgoing direction
                    Vector3 wi;

                    Vector3 n = ray.getNormal();
                    Vector3 newThroughput;

                    // specular caused by IOR
                    float cosTheta = std::abs(ray.getDir().dot(ray.getNormal()));
                    float R0 = surfaceIntergrator->fresnelSchlickRefraction(cosTheta, mat->IOR); // reflection portion
                    float randomSample2 = dist(rng); // a second sample
                    // ----------------------

                    if (ray.getInternal()) {
                        // Refraction
                        // Continue refraction from previous bounce
                        wi = directionSampler->RefractionDirection(ray, *hitObject);
                        newThroughput = surfaceIntergrator->computeThroughput(wo, wi, mat, n, R0, refreaction, true);

                    } else {
                        if (randomSample <= p_specular) {
                            // Specular (Metallic)
                            wi = directionSampler->SpecularDirection(ray, *hitObject, false);
                            newThroughput = surfaceIntergrator->computeThroughput(wo, wi, mat, n, R0, metallic, false);
                        } else if (randomSample <= p_specular + p_transmission) {
                            // Blend in the possibility of refraction based on (transmission * (1 - metallic))
                            if (randomSample2 < R0) {
                                // Specular (Glass)
                                wi = directionSampler->SpecularDirection(ray, *hitObject, false);
                                newThroughput = surfaceIntergrator->computeThroughput(wo, wi, mat, n, R0, specularFresnel, false);
                            } else {
                                // Refraction
                                wi = directionSampler->RefractionDirection(ray, *hitObject);
                                newThroughput = surfaceIntergrator->computeThroughput(wo, wi, mat, n, R0, refreaction, false);
                            }
                        } else if (randomSample <= p_specular + p_transmission + p_diffuse) {
                            if (randomSample2 < R0) {
                                // Specular Diffuse
                                wi = directionSampler->SpecularDirection(ray, *hitObject, false);
                                newThroughput = surfaceIntergrator->computeThroughput(wo, wi, mat, n, R0, specularFresnel, false);
                            } else {
                                // Dielectric reflection
                                wi = directionSampler->DiffuseDirection(ray, *hitObject, false); // sample direction
                                newThroughput = surfaceIntergrator->computeThroughput(wo, wi, mat, n, R0, diffuse, false);
                            }
                        } else {
                            std::cout << "Probabilities dont add up: " << randomSample<<std::endl;
                            break;
                        }
                    }

                    // 2) Add emission *through* the throughput
                    finalColour = finalColour + throughput * (mat->colour * mat->emission);

                    // 3) Update throughput with BRDF and PDF
                    throughput = throughput * newThroughput;

                    /*float dotProduct = std::abs(ray.getNormal().dot(ray.getDir()));
                    Vector3 surfaceReflectance = mat.colour;
                    throughput = throughput * surfaceReflectance * dotProduct;*/

                    // 4) Russian roulette or other bounce termination can go here
                    //    For now, we just rely on bounceDepth or no intersection
                    //    but you can add a random termination, e.g.:
                    // float rr = dist(rng);
                    // if (rr > 0.9f) break;
                    // else throughput *= 1.0f / 0.9f;

                    ray.getDir().set(wi);
                    ray.updateOrigin(0.01);
                } // end for bounceDepth

                // -------------------------------------------------------------
                // Ray terminated. Accumulate into the buffer.
                // -------------------------------------------------------------
                hdrR[y * internalResX + x] += finalColour.x;
                hdrG[y * internalResX + x] += finalColour.y;
                hdrB[y * internalResX + x] += finalColour.z;

                // Update progressive buffer
                lumR[y * internalResX + x] = hdrR[y * internalResX + x] / (
                                                 static_cast<float>(currentRay) + static_cast<float>(its) * static_cast<float>(config.raysPerPixel));
                lumG[y * internalResX + x] = hdrG[y * internalResX + x] / (
                                                 static_cast<float>(currentRay) + static_cast<float>(its) * static_cast<float>(config.raysPerPixel));
                lumB[y * internalResX + x] = hdrB[y * internalResX + x] / (
                                                 static_cast<float>(currentRay) + static_cast<float>(its) * static_cast<float>(config.raysPerPixel));
            } // end raysPerPixel
        } // end x
    } // end y
}

void CPUPT::toneMap(float maxLuminance, int xstart, int xend, int ystart, int yend, std::mutex &mutex) {
    for (int x = xstart; x <= xend; x++) {
        for (int y = ystart; y <= yend; y++) {
            float red = lumR[y * internalResX + x];
            float green = lumG[y * internalResX + x];
            float blue = lumB[y * internalResX + x];

            float luminance = 0.2126f * red + 0.7152f * green + 0.0722f * blue;

            if (luminance > 0) {
                // Extended Reinhard Tone Mapping - returns value [0, 1]
                float mappedLuminance = (luminance * (1 + (luminance / (maxLuminance * maxLuminance)))) / (1 + luminance);

                red = red / luminance * mappedLuminance;
                green = green / luminance * mappedLuminance;
                blue = blue / luminance * mappedLuminance;

                // Apply gamma correction
                float gamma = 2.2f;
                float invGamma = 1.0f / gamma;
                red = pow(red, invGamma);
                green = pow(green, invGamma);
                blue = pow(blue, invGamma);

                red *= 255;
                green *= 255;
                blue *= 255;

                red = std::min(red, 255.0f);
                green = std::min(green, 255.0f);
                blue = std::min(blue, 255.0f);
            }

            // upscale and store in RGB buffer considering aspect ratio
            for (int i = 0; i < upScale; i++) {
                for (int j = 0; j < upScale; j++) {
                    int outX = static_cast<int>(x * upScale + i);
                    int outY = static_cast<int>(y * upScale + j);
                    int offset = (outY * resX + outX) * 3;

                    RGBBuffer[offset] = red;
                    RGBBuffer[offset + 1] = green;
                    RGBBuffer[offset + 2] = blue;
                }
            }
        }
    }
}

void CPUPT::initialiseObjects() {

    resX = config.resX;
    resY = config.resY;

    upScale = config.upScale;

    internalResX = resX / upScale;
    internalResY = resY / upScale;

    aspectRatio = static_cast<float>(resX) / resY;

    RGBBuffer = new uint8_t[resX * resY * 3];

    int res = internalResX * internalResY;
    lumR.resize(res, 0.0f);
    lumG.resize(res, 0.0f);
    lumB.resize(res, 0.0f);
    hdrR.resize(res, 0.0f);
    hdrG.resize(res, 0.0f);
    hdrB.resize(res, 0.0f);
}

void CPUPT::updateUpscaling() {
    upScale = config.upScale;

    internalResX = config.resX / config.upScale;
    internalResY = resY / config.upScale;

    int res = internalResX * internalResY;

    lumR.resize(res, 0.0f);
    lumG.resize(res, 0.0f);
    lumB.resize(res, 0.0f);
    hdrR.resize(res, 0.0f);
    hdrG.resize(res, 0.0f);
    hdrB.resize(res, 0.0f);
}

void CPUPT::deleteObjects() {
    delete[] RGBBuffer;
    delete directionSampler;
    delete surfaceIntergrator;
}

SceneObject* CPUPT::getClickedObject(int screenX, int screenY) {

    int x = screenX / upScale;
    int y = screenY / upScale;

    // Ray cast to find clicked object
    Ray ray;
    ray.reset();
    // Camera (primary) ray origin:

    ray.getOrigin().set(camera->getPos());
    ray.getPos().set(camera->getPos());

    // Jitter the pixel position for MSAA
    float jitterX = (static_cast<float>(rand()) / RAND_MAX - 0.5f) / internalResX;
    float jitterY = (static_cast<float>(rand()) / RAND_MAX - 0.5f) / internalResY;
    Vector3 pixelPosPlane(((((x + 0.5f + jitterX) / internalResX) * 2) - 1) * aspectRatio,
                          1 - (((y + 0.5f + jitterY) / internalResY) * 2), 0);
    Vector3 pixelPosScene(pixelPosPlane.getX() * camera->getPlaneWidth() / 2,
                          pixelPosPlane.getY() * camera->getPlaneHeight() / 2, 0);


    // Point ray according to pixel position (ray starts from camera origin)
    ray.getDir().set(camera->getDir() + camera->getRight() * pixelPosScene.getX() + camera->getUp() * pixelPosScene.getY());
    ray.getDir().normalise();

    // Search BVH
    BVHNode::BVHResult leafNode = rootNode->searchBVHTreeScene(ray);

    return leafNode.node->getSceneObject();
}

std::pair<int, int> CPUPT::threadSegments(float start, float end, int &numThreads, int step) {
    int res = end - start;
    int pixelsPerThread = res / numThreads; // number of pixels per thread
    int remainder = res % numThreads; // remaining pixels after division

    int startPixel = step * pixelsPerThread + std::min(step, remainder);
    int endPixel = (step + 1) * pixelsPerThread + std::min(step + 1, remainder) - 1;

    return std::make_pair(startPixel, endPixel);
}