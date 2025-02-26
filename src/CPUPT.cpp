#include "CPUPT.h"

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"

#include "Window.h"
#include "Renderer.h"

#include <atomic>
#include <algorithm>
#include <chrono>
#include <future>
#include <iostream>
#include <thread>
#include <mutex>
#include <numbers>
#include <stack>
#include <vector>

#include "Config.h"

#include "Ray.h"
#include "BVHNode.h"
#include "UI.h"

#include "SystemManager.h"

CPUPT::CPUPT(SystemManager* systemManager) : systemManager(systemManager), dist(0.0f, 1.0f), iterations(1), numThreads(0) {

}

void CPUPT::launchRenderThread(std::vector<SceneObject *> &sceneObjectsList) {

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
    // create all ray and luminance vector objects
    std::cout << "Constructing Objects: " << std::endl;
    initialiseObjects();
    std::cout << "Avaliable Threads: " << std::thread::hardware_concurrency() << std::endl;
    config.threads = std::thread::hardware_concurrency();

    std::cout << "Constructing BVH: " << std::endl;
    constructBVHST(sceneObjectsList);
    //constructLinearBVH(sceneobjectsList);
    std::cout << "Finished constructing BVH: " << std::endl;

    // render loop code
    while (systemManager->getIsRunning()) {
        auto frameStartTime = std::chrono::high_resolution_clock::now();

        numThreads = config.threads > 0 ? config.threads : std::thread::hardware_concurrency();
        int segments = std::round(std::sqrt(numThreads));

        if (UI::camUpdate || camera->getCamMoved() || !UI::accumulateRays) {

            UI::camUpdate = false;
            camera->getCamMoved() = false;

            if (UI::sceneUpdate) {
                UI::sceneUpdate = false;
                constructBVHST(sceneObjectsList);
            }

            if (UI::resUpdate) {
                updateResolution(); // change buffer size
                UI::resUpdate = false;
                UI::resizeBuffer = true;
                camera->reInitilize();
            }

            // reset colour buffer
            for (int i = 0; i < internalResX * internalResY; i++) {
                lumR[i] = 0.0f;
                lumG[i] = 0.0f;
                lumB[i] = 0.0f;
                absR[i] = 0.0f;
                absG[i] = 0.0f;
                absB[i] = 0.0f;
            }
            iterations = 1;
            UI::accumulatedRays = iterations * config.raysPerPixel;
        }

        if (UI::upscalingUpdate) {
            UI::upscalingUpdate = false;
            config.upScale = UI::upscale;
            updateUpscaling();
        }

        Camera cameraCopy = *camera; // dereference camera and copy


        auto startTimeRays = std::chrono::high_resolution_clock::now();
        for (int j = 0; j < segments; j++) {
            for (int i = 0; i < segments; i++) {
                boundsX = threadSegments(0, internalResX, segments, i);
                boundsY = threadSegments(0, internalResY, segments, j);
                int its = iterations;
                threads.emplace_back(std::async(std::launch::async, &CPUPT::traceRay, this,
                                                cameraCopy, boundsX.first, boundsX.second, boundsY.first,
                                                boundsY.second, its, std::ref(mutex)));
            }
        }
        for (std::future<void> &thread: threads) {
            thread.get(); // Blocks until the thread completes its task
        }
        threads.clear();

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
        maxLuminance *= config.ISO;

        auto startTimeTM = std::chrono::high_resolution_clock::now();
        for (int j = 0; j < segments; j++) {
            for (int i = 0; i < segments; i++) {
                boundsX = threadSegments(0, internalResX, segments, i);
                boundsY = threadSegments(0, internalResY, segments, j);
                threads.emplace_back(std::async(std::launch::async, &CPUPT::toneMap, this, maxLuminance,
                                                boundsX.first, boundsX.second, boundsY.first,
                                                boundsY.second, std::ref(mutex)));
            }
        }
        for (std::future<void> &thread : threads) {
            thread.get(); // Blocks until the thread completes its task
        }
        threads.clear();

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

void CPUPT::traceRay(Camera camera, int xstart, int xend, int ystart, int yend, int its, std::mutex &mutex) const {
    Vector3 lum;
    Vector3 col;
    Ray ray;

    for (int y = ystart; y <= yend; y++) {
        for (int x = xstart; x <= xend; x++) {
            for (int currentRay = 1; currentRay <= config.raysPerPixel; currentRay++) {
                ray.reset();
                std::vector<BounceInfo> bounceInfo;
                for (int currentBounce = 0; currentBounce <= config.bounceDepth; currentBounce++) {
                    if (currentBounce == 0) {
                        //std::cout<<"Primary Ray"<<std::endl;
                        // primary Ray
                        ray.getOrigin().set(camera.getPos());
                        ray.getPos().set(camera.getPos());
                        // calculate position of pixel on image plane
                        // jitter the pixel position for MSAA
                        //std::cout<<"Rand"<<std::endl;
                        float jitterX = (static_cast<float>(rand()) / RAND_MAX - 0.5f) / internalResX;
                        float jitterY = (static_cast<float>(rand()) / RAND_MAX - 0.5f) / internalResY;
                        Vector3 pixelPosPlane(((((x + 0.5f + jitterX) / internalResX) * 2) - 1) * aspectRatio,
                                              1 - (((y + 0.5f + jitterY) / internalResY) * 2), 0);
                        Vector3 pixelPosScene(pixelPosPlane.getX() * camera.getPlaneWidth() / 2, pixelPosPlane.getY() * camera.getPlaneHeight() / 2, 0);
                        // point ray according to pixel position (ray starts from camera origin)
                        //std::cout<<"Setting ray direction"<<std::endl;
                        ray.getDir().set(camera.getDir() + camera.getRight() * pixelPosScene.getX() + camera.getUp() * pixelPosScene.getY());
                        ray.getDir().normalise();
                        if (config.DepthOfField) {
                            // compute Focal Point
                            Vector3 focalPoint = camera.getPos() + ray.getDir() * config.focalDistance;
                            // Randomly sample a point within the aperture
                            float lensU = ((static_cast<float>(rand()) / RAND_MAX) - 0.5f) * 2.0f * config.apertureRadius;
                            float lensV = ((static_cast<float>(rand()) / RAND_MAX) - 0.5f) * 2.0f * config.apertureRadius;
                            Vector3 lensOffset = camera.getRight() * lensU + camera.getUp() * lensV;
                            ray.getOrigin().set(camera.getPos() + lensOffset);
                            ray.getPos().set(camera.getPos() + lensOffset);
                            ray.getDir().set(focalPoint - ray.getOrigin());
                            ray.getDir().normalise();
                        }
                    } else {
                        // secondary Ray
                        //std::cout<<"Secondary Ray"<<std::endl;
                        float randomSample = dist(rng); // monte carlo sampling
                        if (randomSample >= ray.getHitObject()->getMaterial().transmission) {
                            sampleReflectionDirection(ray, *ray.getHitObject(), false);
                        } else {
                            sampleRefractionDirection(ray, *ray.getHitObject(), false);
                        }
                    }
                    //std::cout<<"Traversing BVH"<<std::endl;
                    BVHNode::BVHResult leafNode = BVHNodes.at(0)->searchBVHTreeScene(ray);
                    //std::cout<<"Finished Traversing BVH"<<std::endl;
                    if (leafNode.node != nullptr && leafNode.node->getLeaf()) {
                        //std::cout<<"Scene Object Found"<<std::endl;
                        SceneObject *BVHSceneObject = leafNode.node->getSceneObject();
                        ray.march(leafNode.close);
                        ray.getHitPoint().set(ray.getPos());
                        ray.setHitObject(BVHSceneObject);
                        ray.getOrigin().set(ray.getPos());
                        // store hit data
                        BVHSceneObject->getNormal(ray); // update normal vector
                        BounceInfo currentBounceInfo{};
                        currentBounceInfo.emission = BVHSceneObject->getMaterial().emission;
                        currentBounceInfo.colour = BVHSceneObject->getMaterial().colour;
                        currentBounceInfo.metallic = BVHSceneObject->getMaterial().metallic;
                        currentBounceInfo.dot = std::abs(ray.getNormal().dot(ray.getDir()));
                        bounceInfo.push_back(currentBounceInfo);
                    }
                    else {
                        currentBounce = config.bounceDepth; // end ray
                    }
                }
                float red = 0;
                float green = 0;
                float blue = 0;
                for (int index = bounceInfo.size() - 1; index >= 0; index--) {
                    // Mix between diffuse (non-metallic) and specular (metallic) based on metallic value
                    float dotProduct = bounceInfo[index].dot;  // Ensure this is cosine of the angle
                    float metallic = bounceInfo[index].metallic;
                    Vector3 baseColour = bounceInfo[index].colour;

                    red = (bounceInfo[index].emission + red) * baseColour.x * dotProduct;
                    green = (bounceInfo[index].emission + green) * baseColour.y * dotProduct;
                    blue = (bounceInfo[index].emission + blue) * baseColour.z * dotProduct;
                }

                absR[y * internalResX + x] += red;
                absG[y * internalResX + x] += green;
                absB[y * internalResX + x] += blue;
                lumR[y * internalResX + x] = absR[y * internalResX + x] / (static_cast<float>(currentRay) * static_cast<float>(its));
                lumG[y * internalResX + x] = absG[y * internalResX + x] / (static_cast<float>(currentRay) * static_cast<float>(its));
                lumB[y * internalResX + x] = absB[y * internalResX + x] / (static_cast<float>(currentRay) * static_cast<float>(its));
            }
        }
    }
}

thread_local std::mt19937 CPUPT::rng(std::random_device{}());

void CPUPT::sampleRefractionDirection(Ray &ray, const SceneObject &sceneObject, bool flipNormal) const {
    float n1 = 1.0003f; // refractive index of air
    float n2 = sceneObject.getMaterial().IOR;
    // cosine of incient angle
    float cosTheta1 = -(ray.getNormal().dot(ray.getDir())); // negative of the dot product because normal may be pointing inwards
    float sinTheta1 = std::sqrt(1.0f - cosTheta1 * cosTheta1);
    float sinTheta2 = (n1 / n2) * sinTheta1;

    if (sinTheta2 >= 1) {
        // total internal reflection - bounce off object
        sampleReflectionDirection(ray, sceneObject, false);
    } else {
        // valid refreaction into next medium
        float cosTheta2 = std::sqrt(1.0f - sinTheta2 * sinTheta2);
        sceneObject.getNormal(ray); // update normal
        Vector3 refraction(ray.getDir() * (n1 / n2) + ray.getNormal() * ((n1 / n2) * cosTheta1 - cosTheta2));

        ray.getDir().set(refraction);
        ray.getDir().normalise();
        ray.updateOrigin(sceneObject.getIntersectionDistance(ray).second); // march the ray to the other side of the object
        n1 = sceneObject.getMaterial().IOR;
        n2 = 1.0003;
        // cosine of incident angle
        sceneObject.getNormal(ray); // update normal
        cosTheta1 = -(ray.getNormal().dot(ray.getDir()));
        sinTheta1 = std::sqrt(1.0f - cosTheta1 * cosTheta1);
        sinTheta2 = (n1 / n2) * sinTheta1;
        int totalInternalReflections = 0; // cap on total interal reflections
        while (sinTheta2 >= 1 && totalInternalReflections <= 5) {
            sampleReflectionDirection(ray, sceneObject, true); // inside object so flipped normal
            ray.updateOrigin(-0.01f); // undo the march from the previous method
            ray.updateOrigin(sceneObject.getIntersectionDistance(ray).second); // march the ray to the other side of the object
            // recalculate the sin of the angle to work out if the ray still has total internal reflection or not
            sceneObject.getNormal(ray); // update normal
            cosTheta1 = -(ray.getNormal().dot(ray.getDir()));
            sinTheta1 = std::sqrt(1.0f - cosTheta1 * cosTheta1);
            sinTheta2 = (n1 / n2) * sinTheta1;
            totalInternalReflections++;
        }
        cosTheta2 = std::sqrt(1.0f - sinTheta2 * sinTheta2);
        sceneObject.getNormal(ray); // update normal
        refraction.set(ray.getDir() * (n1 / n2) + ray.getNormal() * ((n1 / n2) * cosTheta1 - cosTheta2));
        ray.getDir().normalise();
        ray.updateOrigin(sceneObject.getIntersectionDistance(ray).second + 0.01f);
    }
    ray.setBounceDot(1.0f);
}

void CPUPT::sampleReflectionDirection(Ray &ray, const SceneObject &sceneObject, bool flipNormal) const {
    float roughness = sceneObject.getMaterial().roughness;
    // reflection direction
    float dotProduct = ray.getDir().dot(ray.getNormal());
    Vector3 reflection(ray.getDir() - ray.getNormal() * dotProduct * 2);

    if (roughness > 0) {
        if (flipNormal) {
            ray.getNormal().flip();
        }

        // generate random direction
        // two randoms between 0 and 1
        float alpha = dist(rng);
        float gamma = dist(rng);
        // convert to sphereical coodinates
        alpha = std::acos(std::sqrt(alpha)); // polar angle - sqrt more likely to be near the pole (z axis)
        gamma = 2.0f * std::numbers::pi * gamma; // azimuthal angle

        // convert random direction in spherical coordinates to vector coordinates
        Vector3 random(
            std::sin(alpha) * std::cos(gamma),
            std::sin(alpha) * std::sin(gamma),
            std::cos(alpha));
        random.normalise();

        // convert to tangent space (a coordinate system defined by the normal of the surface)
        // calculate Tangent and Bitangnet vectors using arbitrary vector a
        Vector3 arbitraryA;
        // if the normals are exactly 0 there are problems... if statement to catch that
        if (std::abs(ray.getNormal().getX()) > 0.0001 || std::abs(ray.getNormal().getZ()) > 0.0001) {
            arbitraryA.set(0, 1, 0);
        } else { arbitraryA.set(1, 0, 0); }
        // tangent vector T equals cross product of normal N and arbitrary vector a
        Vector3 tangentT(ray.getNormal().cross(arbitraryA));
        tangentT.normalise();

        // bitangnet vector B equals cross product of tangent and normal
        Vector3 bitangent(ray.getNormal().cross(tangentT));
        bitangent.normalise();

        // set final sampled direction
        // x = randomX * tangentX + randomY * bitangentX + randomZ * normalX
        Vector3 diffuseD(
            random.getX() * tangentT.getX() + random.getY() * bitangent.getX() + random.getZ() * ray.getNormal().getX(),
            random.getX() * tangentT.getY() + random.getY() * bitangent.getY() + random.getZ() * ray.getNormal().getY(),
            random.getX() * tangentT.getZ() + random.getY() * bitangent.getZ() + random.getZ() * ray.getNormal().getZ());

        // bias the reflection direction with the random direction
        // biasedDirection = (1 - roughness) * reflectionDirection + roughness * randomDirection

        ray.getDir().set(
            ((1 - roughness) * reflection.getX()) + roughness * diffuseD.getX(),
            ((1 - roughness) * reflection.getY()) + roughness * diffuseD.getY(),
            ((1 - roughness) * reflection.getZ()) + roughness * diffuseD.getZ());
    } else {
        // pure reflection direction if rough = 0
        ray.getDir().set(reflection.getX(), reflection.getY(), reflection.getZ());
    }
    ray.getDir().normalise();
    float bounceDot = ray.getDir().dot(ray.getNormal());
    if (bounceDot < 0) {
        ray.getDir().flip();
        ray.setBounceDot(-bounceDot);
    } else {
        ray.setBounceDot(bounceDot);
    }
    ray.updateOrigin(0.01f); // march the ray a tiny amount to move it off the object
}

void CPUPT::constructBVHST(const std::vector<SceneObject *> &sceneObjectsList) {
    // create leaf nodes
    for (SceneObject *sceneObject: sceneObjectsList) {
        std::pair<Vector3, Vector3> bounds = sceneObject->getBounds();
        BoundingBox *boundingBox = new BoundingBox(bounds.first, bounds.second);
        BVHNodes.emplace_back(new BVHNode(boundingBox, *sceneObject));
        BVHNodes.at(BVHNodes.size() - 1)->setLeaf(true);
    }

    std::cout << "Number of leaf nodes: " << BVHNodes.size() << std::endl;

    // create tree structure from lead nodes - bottom up
    auto startTime = std::chrono::high_resolution_clock::now();
    while (BVHNodes.size() > 1) {
        float cost = 0, bestCost = std::numeric_limits<float>::infinity();
        int indexLeft = 0, indexRight = 0;
        BVHNode *bestLeft = nullptr;
        BVHNode *bestRight = nullptr;
        BoundingBox combinedBox;

        for (int i = 0; i < BVHNodes.size(); i++) {
            for (int j = i + 1; j < BVHNodes.size(); j++) {
                combinedBox.updateBounds(*BVHNodes.at(i)->getBoundingBox(), *BVHNodes.at(j)->getBoundingBox());
                cost = (combinedBox.getArea() / (BVHNodes.at(i)->getArea() + BVHNodes.at(j)->getArea())) * (
                           BVHNodes.at(i)->getNumChildren() + BVHNodes.at(j)->getNumChildren());

                if (cost < bestCost) {
                    bestCost = cost;
                    bestLeft = BVHNodes.at(i);
                    bestRight = BVHNodes.at(j);
                    indexLeft = i;
                    indexRight = j;
                }
            }
        }
        // create a new BVHNode that has the smallest combined area
        BoundingBox *parentBox = new BoundingBox(*bestLeft->getBoundingBox(), *bestRight->getBoundingBox());
        BVHNode *parentNode = new BVHNode(parentBox, bestLeft, bestRight);
        BVHNodes.emplace_back(parentNode);
        // erase node from list
        if (indexLeft > indexRight) {
            BVHNodes.erase(BVHNodes.begin() + indexLeft);
            BVHNodes.erase(BVHNodes.begin() + indexRight);
        } else {
            BVHNodes.erase(BVHNodes.begin() + indexRight);
            BVHNodes.erase(BVHNodes.begin() + indexLeft);
        }
    }
    auto stopTime = std::chrono::high_resolution_clock::now();
    auto durationTime = std::chrono::duration_cast<std::chrono::microseconds>(stopTime - startTime);
    std::cout << "Finished tree creation: " << durationTime.count() << "us" << std::endl;
    //std::cout << "BVHNode size: " << BVHNodes.size() << std::endl;
    std::cout << "RootNode numChildren: " << BVHNodes.at(0)->getNumChildren() << std::endl;
}

void CPUPT::constructBVHMT(const std::vector<SceneObject *> &sceneObjectsList) {
    // create leaf nodes
    for (SceneObject *sceneObject: sceneObjectsList) {
        std::pair<Vector3, Vector3> bounds = sceneObject->getBounds();
        BoundingBox *boundingBox = new BoundingBox(bounds.first, bounds.second);
        BVHNodes.emplace_back(new BVHNode(boundingBox, *sceneObject));
        BVHNodes.at(BVHNodes.size() - 1)->setLeaf(true);
    }
    std::cout << "Number of leaf nodes: " << BVHNodes.size() << std::endl;

    auto startTime = std::chrono::high_resolution_clock::now();

    size_t numThreads = std::thread::hardware_concurrency();
    std::cout << "Threads avaliable: " << numThreads << std::endl;
    //std::vector<std::thread> threads;
    std::vector<std::future<void> > threads;
    std::mutex mutex;

    while (BVHNodes.size() > 1) {
        size_t numNodes = BVHNodes.size();
        size_t nodesPerThread = numNodes / numThreads; // basic number of nodes per thread
        size_t remainder = numNodes % numThreads; // remaining nodes after even distribution

        std::atomic<float> bestCost = std::numeric_limits<float>::infinity();
        BVHNode *bestLeft = nullptr, *bestRight = nullptr;
        int indexLeft = 0, indexRight = 0;
        if (numNodes > numThreads) {
            for (size_t i = 0; i < numThreads; i++) {
                size_t start = i * nodesPerThread + std::min(i, remainder);
                size_t end = start + nodesPerThread + (i < remainder ? 1 : 0);

                end = std::min(end, numNodes); // ensure last thread does not exceed size of nodes vector
                threads.emplace_back(std::async(std::launch::async, &CPUPT::findBestPair, this,
                                                std::ref(BVHNodes), start, end, std::ref(bestCost), std::ref(indexLeft), std::ref(indexRight),
                                                std::ref(bestLeft), std::ref(bestRight), std::ref(mutex)));
            }
        } else {
            for (int i = 0; i < BVHNodes.size(); i++) {
                int end = i + 1;
                threads.emplace_back(std::async(std::launch::async, &CPUPT::findBestPair, this,
                                                std::ref(BVHNodes), i, end, std::ref(bestCost), std::ref(indexLeft), std::ref(indexRight),
                                                std::ref(bestLeft),
                                                std::ref(bestRight), std::ref(mutex)));
            }
        }

        for (std::future<void> &thread: threads) {
            thread.get(); // Blocks until the thread completes its task
        }
        threads.clear();

        // create a new BVHNode that has the smallest combined area
        BoundingBox *parentBox = new BoundingBox(*bestLeft->getBoundingBox(), *bestRight->getBoundingBox());
        BVHNode *parentNode = new BVHNode(parentBox, bestLeft, bestRight);
        BVHNodes.emplace_back(parentNode);
        // erase node from list
        if (indexLeft > indexRight) {
            BVHNodes.erase(BVHNodes.begin() + indexLeft);
            BVHNodes.erase(BVHNodes.begin() + indexRight);
        } else {
            BVHNodes.erase(BVHNodes.begin() + indexRight);
            BVHNodes.erase(BVHNodes.begin() + indexLeft);
        }
    }

    auto stopTime = std::chrono::high_resolution_clock::now();
    auto durationTime = std::chrono::duration_cast<std::chrono::microseconds>(stopTime - startTime);
    std::cout << "Finished tree creation: " << durationTime.count() << "us" << std::endl;
    std::cout << "BVHNode size: " << BVHNodes.size() << std::endl;
    std::cout << "RootNode numChildren: " << BVHNodes.at(0)->getNumChildren() << std::endl;
}

void CPUPT::findBestPair(const std::vector<BVHNode *> &nodes, int start, int end, std::atomic<float> &globalBestCost, int &leftIndex, int &rightIndex,
                          BVHNode *&bestLeft, BVHNode *&bestRight, std::mutex &mutex) {
    float localBestCost = std::numeric_limits<float>::infinity();
    BVHNode *localBestLeft = nullptr, *localBestRight = nullptr;
    int localIndexLeft = 0, localIndexRight = 0;

    for (int i = start; i < end; i++) {
        for (int j = i + 1; j < nodes.size(); j++) {
            BoundingBox *combinedBox = new BoundingBox(*nodes[i]->getBoundingBox(), *nodes[j]->getBoundingBox());
            float cost = (combinedBox->getArea() / (nodes[i]->getArea() + nodes[j]->getArea())) * (nodes[i]->getNumChildren() + nodes[j]->getNumChildren());
            if (cost < localBestCost) {
                localBestCost = cost;
                localBestLeft = nodes[i];
                localBestRight = nodes[j];
                localIndexLeft = i;
                localIndexRight = j;
            }
            delete combinedBox;
        }
    }
    std::lock_guard<std::mutex> guard(mutex);
    if (localBestCost < globalBestCost) {
        globalBestCost = localBestCost;
        bestLeft = localBestLeft;
        bestRight = localBestRight;
        leftIndex = localIndexLeft;
        rightIndex = localIndexRight;
    }
}

void CPUPT::initialiseObjects() {

    camera = systemManager->getCamera();

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
    absR.resize(res, 0.0f);
    absG.resize(res, 0.0f);
    absB.resize(res, 0.0f);
}

void CPUPT::updateUpscaling() {
    upScale = config.upScale;
    internalResX = config.resX / config.upScale;
    internalResY = resY / config.upScale;
    int res = internalResX * internalResY;
    lumR.resize(res, 0.0f);
    lumG.resize(res, 0.0f);
    lumB.resize(res, 0.0f);
    absR.resize(res, 0.0f);
    absG.resize(res, 0.0f);
    absB.resize(res, 0.0f);
}

void CPUPT::updateResolution() {

    std::cout<<"Buffers Resized"<<std::endl;
    resX = config.resX;
    resY = config.resY;

    internalResX = config.resX / config.upScale;
    internalResY = config.resY / config.upScale;

    aspectRatio = static_cast<float>(internalResX) / internalResY;

    //delete[] RGBBuffer;
    RGBBuffer = new uint8_t[resX * resY * 3];

    int res = internalResX * internalResY;
    lumR.resize(res, 0.0f);
    lumG.resize(res, 0.0f);
    lumB.resize(res, 0.0f);
    absR.resize(res, 0.0f);
    absG.resize(res, 0.0f);
    absB.resize(res, 0.0f);
}

void CPUPT::deleteObjects() {
    for (auto node: BVHNodes) {
        delete node;
    }
    BVHNodes.clear();

    delete[] RGBBuffer;
}

std::pair<int, int> CPUPT::threadSegments(float start, float end, int &numThreads, int step) {
    int res = end - start;
    int pixelsPerThread = res / numThreads; // number of pixels per thread
    int remainder = res % numThreads; // remaining pixels after division

    int startPixel = step * pixelsPerThread + std::min(step, remainder);
    int endPixel = (step + 1) * pixelsPerThread + std::min(step + 1, remainder) - 1;

    return std::make_pair(startPixel, endPixel);
}

void CPUPT::constructLinearBVH(const std::vector<SceneObject *> &sceneObjectsList) {
    // create a leaf node for each scene object
    for (int i = 0; i < sceneObjectsList.size(); i++) {
        std::pair<Vector3, Vector3> bounds = sceneObjectsList[i]->getBounds();
        LinearBVHNode leafNode;
        leafNode.bounds = BoundingBox(bounds.first, bounds.second);
        leafNode.objectIndex = i;
        leafNode.isLeaf = true;
        leafNode.leftChild = -1;
        leafNode.rightChild = -1;
        leafNode.numChildren = 1;
        bvhNodes.push_back(leafNode);
    }

    // a list of indexs to the bvhNodes vector for nodes that have not yet been merged
    std::vector<int> activeNodes;
    for (int i = 0; i < bvhNodes.size(); i++) {
        activeNodes.push_back(i);
    }

    while (activeNodes.size() > 1) {
        // use SAH to find best pair
        float cost = 0, bestCost = std::numeric_limits<float>::infinity();
        BoundingBox combinedBox{};
        int bestLeft = -1, bestRight = -1;

        for (int i = 0; i < activeNodes.size(); i++) {
            for (int j = i + 1; j < activeNodes.size(); j++) {
                int left = activeNodes[i];
                int right = activeNodes[j];
                combinedBox.updateBounds(bvhNodes[left].bounds, bvhNodes[right].bounds);
                cost = (combinedBox.getArea() / (bvhNodes[left].bounds.getArea() + bvhNodes[right].bounds.getArea())) * (
                           bvhNodes[left].numChildren + bvhNodes[right].numChildren);
                if (cost < bestCost) {
                    bestCost = cost;
                    bestLeft = i;
                    bestRight = j;
                }
            }
        }

        // Merge the two best nodes
        LinearBVHNode parentNode;
        combinedBox.updateBounds(bvhNodes[activeNodes[bestLeft]].bounds, bvhNodes[activeNodes[bestRight]].bounds);
        parentNode.bounds = BoundingBox(combinedBox);
        parentNode.isLeaf = false;
        parentNode.leftChild = activeNodes[bestLeft];
        parentNode.rightChild = activeNodes[bestRight];
        parentNode.numChildren = bvhNodes[activeNodes[bestLeft]].numChildren + bvhNodes[activeNodes[bestRight]].numChildren;

        int parentIndex = bvhNodes.size();
        bvhNodes.push_back(parentNode); // add the new parent node to the list

        // remove the merged nodes from activeNodes and add the new parent node
        if (bestLeft < bestRight) {
            std::swap(bestLeft, bestRight); // Ensure bestLeft is always smaller
        }
        activeNodes.erase(activeNodes.begin() + bestLeft); // Erase the larger index first
        activeNodes.erase(activeNodes.begin() + bestRight); // Then erase the smaller index
        activeNodes.push_back(parentIndex);
    }
}

CPUPT::BVHResult CPUPT::searchLinearBVH(Ray &ray, const std::vector<SceneObject *> &sceneObjectsList) const {
    std::stack<int> nodeStack; // list of nodes to check, by index to node vector
    nodeStack.push(bvhNodes.size() - 1); // start at root node

    int closestObject = -1;
    float closestDistance = std::numeric_limits<float>::infinity();
    float closestExit = std::numeric_limits<float>::infinity();

    while (!nodeStack.empty()) {
        int nodeIndex = nodeStack.top();
        nodeStack.pop(); // remove the node we are checking from the list

        const LinearBVHNode &node = bvhNodes[nodeIndex];

        std::pair<float, float> nodeDistance = node.bounds.getIntersectionDistance(ray);
        std::pair<float, float> objDistance = {-1.0f, -1.0f};

        if (!(nodeDistance.first <= nodeDistance.second && nodeDistance.second >= 0)) {
            continue; // if the ray doesnt intersect the nodes bounding box skip it
        }
        if (node.isLeaf) {
            objDistance = sceneObjectsList[node.objectIndex]->getIntersectionDistance(ray);
            if (objDistance.first <= objDistance.second && objDistance.second >= 0) {
                // check if the ray intersects the object
                if (objDistance.first < closestDistance) {
                    // if the intersection point is closer than the previous best, record it
                    closestDistance = objDistance.first;
                    closestExit = objDistance.second;
                    closestObject = node.objectIndex;
                }
            }
        } else {
            // if the node is not a leaf, but we intersect it, add its children to the list but only if its closer than the current best
            if (nodeDistance.first < closestDistance) {
                nodeStack.push(node.rightChild);
                nodeStack.push(node.leftChild);
            }
        }
    }
    if (closestDistance == std::numeric_limits<float>::infinity()) { closestDistance = -1; }

    return BVHResult{closestObject, closestDistance, closestExit}; // return the index of the closest object, and the intersection distance
}