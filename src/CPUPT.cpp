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

CPUPT::CPUPT(SystemManager *systemManager) : systemManager(systemManager), iterations(0), numThreads(0) {
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
    //config.threads = 1;

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

            config.fOV = UI::fOV;
            config.exposure = UI::exposure;
            camera->reInitilize(); // fov changes

            if (UI::sceneUpdate) {
                UI::sceneUpdate = false;
                constructBVHST(sceneObjectsList);
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
        maxLuminance *= config.exposure;

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
        for (std::future<void> &thread: threads) {
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
    // For convenience, define some random distribution for later
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
                    BVHNode::BVHResult leafNode = BVHNodes.at(0)->searchBVHTreeScene(ray);
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
                    Material mat = hitObject->getMaterial();

                    // 1) Sample new direction: reflection or refraction and compute BRDF and PDF
                    float randomSample = dist(rng);
                    float p_specular = mat.metallic;
                    float p_transmission = mat.transmission * (1.0f - mat.metallic);
                    float p_diffuse = 1.0f - (p_specular + p_transmission);
                    // probabilites should add up to 1

                    Vector3 wo = ray.getDir() * -1; // outgoing direction
                    Vector3 wi;

                    Vector3 n = ray.getNormal();
                    Vector3 newThroughput;

                    // specular caused by IOR
                    float cosTheta = std::abs(dot(ray.getDir(), ray.getNormal()));
                    float R0 = fresnelSchlickRefraction(cosTheta, mat.IOR); // reflection portion
                    float randomSample2 = dist(rng); // a second sample
                    // ----------------------

                    if (ray.getInternal()) {
                        // Refraction
                        // Continue refraction from previous bounce
                        wi = sampleRefractionDirection(ray, *hitObject);
                        newThroughput = computeThroughput(wo, wi, mat, n, R0, refreaction, true);

                    } else {
                        if (randomSample < p_specular) {
                            // Specular (Metallic)
                            wi = sampleSpecularDirection(ray, *hitObject, false);
                            newThroughput = computeThroughput(wo, wi, mat, n, R0, metallic, false);
                        } else if (randomSample < p_specular + p_transmission) {
                            // Blend in the possibility of refraction based on (transmission * (1 - metallic))
                            if (randomSample2 < R0) {
                                // Specular (Glass)
                                wi = sampleSpecularDirection(ray, *hitObject, false);
                                newThroughput = computeThroughput(wo, wi, mat, n, R0, specularFresnel, false);
                            } else {
                                // Refraction
                                wi = sampleRefractionDirection(ray, *hitObject);
                                newThroughput = computeThroughput(wo, wi, mat, n, R0, refreaction, false);
                            }
                        } else if (randomSample < p_specular + p_transmission + p_diffuse) {
                            if (randomSample2 < R0) {
                                // Specular Diffuse
                                wi = sampleSpecularDirection(ray, *hitObject, false);
                                newThroughput = computeThroughput(wo, wi, mat, n, R0, specularFresnel, false);
                            } else {
                                // Dielectric reflection
                                wi = sampleDiffuseDirection(ray, *hitObject, false); // sample direction
                                newThroughput = computeThroughput(wo, wi, mat, n, R0, diffuse, false);
                            }
                        } else {
                            std::cout << "Probabilities dont add up" << std::endl;
                            break;
                        }
                    }

                    // 2) Add emission *through* the throughput
                    finalColour = finalColour + throughput * (mat.colour * mat.emission);

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

thread_local std::mt19937 CPUPT::rng(std::random_device{}());

Vector3 CPUPT::sampleRefractionDirection(Ray &ray, SceneObject &sceneObject) const {
    Vector3 wi = ray.getDir();
    Vector3 N = ray.getNormal();

    float n1 = 1.0003f; // refractive index of air
    float n2 = sceneObject.getMaterial().IOR;

    if (ray.getInternal()) {
        // inside glass, flip normal, IOR, and dotProduct
        N = N * -1;
        std::swap(n1, n2);
    }

    float cosThetaI = N.dot(wi);

    // cosine of incient angle
    float sinTheta1 = std::sqrt(std::max(0.0f, 1.0f - cosThetaI * cosThetaI));
    float sinTheta2 = (n1 / n2) * sinTheta1;

    if (sinTheta2 >= 1) {
        // total internal reflection - bounce off object / bounce back inside object
        Vector3 reflection = wi.reflect(N);
        reflection.normalise();
        return reflection;
    }

    // valid refreaction into next medium
    float cosTheta2 = std::sqrt(std::max(0.0f, 1.0f - sinTheta2 * sinTheta2));
    Vector3 refraction = (wi * (n1 / n2)) + (N * ((n1 / n2) * cosThetaI - cosTheta2));

    refraction.normalise(); // Return normalized refracted direction
    // on first refraction flip internal to true
    // second refraction is always on exit - flips to false
    ray.flipInternal();

    return refraction;
}

float CPUPT::distrubtionGGX(const Vector3 &normal, const Vector3 &halfVector, float roughness) const {
    roughness = std::max(roughness, 0.001f);
    float alpha = roughness * roughness;
    float NdotH = std::max(normal.dot(halfVector), 0.0f);
    float denom = (NdotH * NdotH * (alpha - 1.0f) + 1.0f);

    return alpha / (std::numbers::pi * denom * denom);
}

float CPUPT::geometrySchlickGGX(float NdotV, float roughness) const {
    roughness = std::max(roughness, 0.001f);
    float k = (roughness + 1.0f) * (roughness + 1.0f) / 8.0f;
    return NdotV / (NdotV * (1.0f - k) + k);
}

float CPUPT::geometrySmithGGX(const Vector3 &normal, const Vector3 &viewDir, const Vector3 &lightDir, float roughness) const {
    roughness = std::max(roughness, 0.001f);
    float NdotV = std::max(normal.dot(viewDir), 0.0f);
    float NdotL = std::max(normal.dot(lightDir), 0.0f);
    float ggxV = geometrySchlickGGX(NdotV, roughness);
    float ggxL = geometrySchlickGGX(NdotL, roughness);
    return ggxV * ggxL;
}

Vector3 CPUPT::fresnelSchlickSpecular(float cosTheta, Vector3 F0) const {
    return F0 + (Vector3(1.0f) - F0) * std::pow(1.0f - cosTheta, 5.0f);
}

float CPUPT::fresnelSchlickRefraction(float cosTheta, float ior) const {
    float R0 = (ior - 1.0003f) / (ior + 1.0003f);
    R0 = R0 * R0;
    return R0 + (1.0f - R0) * std::pow(1.0f - cosTheta, 5.0f);
}

Vector3 CPUPT::sampleSpecularDirection(Ray &ray, const SceneObject &sceneObject, bool flipNormal) const {
    const Material &mat = sceneObject.getMaterial();
    float rough = std::max(mat.roughness, 0.001f);
    float alpha = rough * rough; // for GGX


    Vector3 N = ray.getNormal();
    if (flipNormal) {
        N.flip();
    }

    // Sample half vector H in tanget space
    float r1 = dist(rng);;
    float r2 = dist(rng);

    float phi = 2.0f * std::numbers::pi * r1;
    // "Smith" or "Heitz" form for cosTheta half
    float cosTheta = std::sqrt((1.0f - r2) / (1.0f + (alpha * alpha - 1.0f) * r2));
    float sinTheta = std::sqrt(std::max(0.0f, 1.0f - cosTheta * cosTheta));

    // Half vector in local tangent space
    Vector3 Ht(sinTheta * std::cos(phi),
               sinTheta * std::sin(phi),
               cosTheta);

    // Build an orthonormal basis around N
    Vector3 arbitraryA;
    if (std::abs(N.x) < 0.5f && std::abs(N.z) < 0.5f) {
        arbitraryA = Vector3(1, 0, 0);
    } else {
        arbitraryA = Vector3(0, 1, 0);
    }
    Vector3 T = N.cross(arbitraryA);
    T.normalise();
    Vector3 B = N.cross(T);
    B.normalise();

    // Transform Ht to world space - cross product
    Vector3 H(
        Ht.x * T.x + Ht.y * B.x + Ht.z * N.x,
        Ht.x * T.y + Ht.y * B.y + Ht.z * N.y,
        Ht.x * T.z + Ht.y * B.z + Ht.z * N.z
    );
    H.normalise();

    // Reflect incoming dir around H to get outgoing direction
    Vector3 V = ray.getDir();
    V.normalise();

    float dotVH = V.dot(H);
    if (dotVH < 0.0f) {
        H = H * -1;
        dotVH = -dotVH;
    }
    Vector3 R = V - H * 2.0f * dotVH; // reflection direction in world space

    R.normalise();

    return R;
}

Vector3 CPUPT::sampleDiffuseDirection(Ray &ray, const SceneObject &sceneObject, bool flipNormal) const {
    // Diffuse sampling - random direction above surface with cosine-weighted distribution

    // Surface normal
    Vector3 N = ray.getNormal();
    if (flipNormal) {
        N.flip();
    }

    // random numbers for hemisphere sampling
    float alpha = dist(rng);
    float gamma = dist(rng);

    // conver to spherical coordinates

    float theta = std::acos(std::sqrt(alpha)); // polar angle
    float phi = 2.0f * std::numbers::pi * gamma; // azimuth

    // Convert spherical coords to Cartesian in local tangent space
    Vector3 localDir(
        std::sin(theta) * std::cos(phi),
        std::sin(theta) * std::sin(phi),
        std::cos(theta)
    );
    localDir.normalise();

    // Build tangent & bitangent from N
    Vector3 arbitraryA;
    if (std::abs(N.getX()) < 0.5f && std::abs(N.getZ()) < 0.5f) {
        arbitraryA.set(1, 0, 0);
    } else {
        arbitraryA.set(0, 1, 0);
    }
    Vector3 tangentT = N.cross(arbitraryA);
    tangentT.normalise();
    Vector3 bitangent = N.cross(tangentT);
    bitangent.normalise();

    // Transform localDir into world space
    Vector3 worldDir(
        localDir.getX() * tangentT.getX() + localDir.getY() * bitangent.getX() + localDir.getZ() * N.getX(),
        localDir.getX() * tangentT.getY() + localDir.getY() * bitangent.getY() + localDir.getZ() * N.getY(),
        localDir.getX() * tangentT.getZ() + localDir.getY() * bitangent.getZ() + localDir.getZ() * N.getZ()
    );

    worldDir.normalise();

    return worldDir;
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

Vector3 CPUPT::sampleRefractionDirectionAll(Ray &ray, SceneObject &sceneObject) const {
    Ray refracRay;
    refracRay.initialize(ray);

    float n1 = 1.0003f; // refractive index of air
    float n2 = sceneObject.getMaterial().IOR;
    // cosine of incient angle
    float cosTheta1 = (refracRay.getNormal().dot(refracRay.getDir()));

    float sinTheta1 = std::sqrt(1.0f - cosTheta1 * cosTheta1);
    float sinTheta2 = (n1 / n2) * sinTheta1;

    if (sinTheta2 >= 1) {
        // total internal reflection - bounce off object
        return sampleSpecularDirection(refracRay, sceneObject, false);
    }

    // valid refreaction into next medium
    float cosTheta2 = std::sqrt(1.0f - sinTheta2 * sinTheta2);
    sceneObject.getNormal(refracRay); // update normal
    Vector3 refraction(refracRay.getDir() * (n1 / n2) + refracRay.getNormal() * ((n1 / n2) * cosTheta1 - cosTheta2));

    refracRay.getDir().set(refraction);
    refracRay.getDir().normalise();
    refracRay.updateOrigin(sceneObject.getIntersectionDistance(refracRay).second); // march the ray to the other side of the object
    n1 = sceneObject.getMaterial().IOR;
    n2 = 1.0003;
    // cosine of incident angle
    sceneObject.getNormal(refracRay); // update normal
    cosTheta1 = (refracRay.getNormal().dot(refracRay.getDir()));
    sinTheta1 = std::sqrt(1.0f - cosTheta1 * cosTheta1);
    sinTheta2 = (n1 / n2) * sinTheta1;

    int totalInternalReflections = 0; // cap on total interal reflections
    while (sinTheta2 >= 1 && totalInternalReflections <= 5) {
        sampleSpecularDirection(refracRay, sceneObject, true); // inside object so flipped normal
        refracRay.updateOrigin(sceneObject.getIntersectionDistance(refracRay).second); // march the ray to the other side of the object
        // recalculate the sin of the angle to work out if the ray still has total internal reflection or not
        sceneObject.getNormal(refracRay); // update normal
        cosTheta1 = ((refracRay.getNormal() * -1).dot(refracRay.getDir()));
        sinTheta1 = std::sqrt(1.0f - cosTheta1 * cosTheta1);
        sinTheta2 = (n1 / n2) * sinTheta1;
        totalInternalReflections++;
    }
    cosTheta2 = std::sqrt(1.0f - sinTheta2 * sinTheta2);
    sceneObject.getNormal(refracRay); // update normal
    refraction.set(refracRay.getDir() * (n1 / n2) + refracRay.getNormal() * ((n1 / n2) * cosTheta1 - cosTheta2));
    refracRay.getDir().normalise();
    refracRay.updateOrigin(sceneObject.getIntersectionDistance(refracRay).second);

    // update ray origin but not direction to preserve wi and wo;
    ray.setOrigin(refracRay.getOrigin());

    return refracRay.getDir();
}
