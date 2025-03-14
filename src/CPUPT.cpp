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

CPUPT::CPUPT(SystemManager *systemManager, std::vector<SceneObject *> &sceneObjectsList) : systemManager(systemManager), sceneObjectsList(sceneObjectsList),
                                                                                           iterations(0), numThreads(0) {
    directionSampler = new DirectionSampler();
    surfaceIntegrator = new SurfaceIntegrator();
    denoiser = new Denoiser();
    toneMapper = new ToneMapper();

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
    std::cout << "Avaliable Threads: " << std::thread::hardware_concurrency() << std::endl;

    // initialise objects
    std::mutex mutex;
    std::vector<std::future<void> > threads;
    std::vector<std::future<void> > threadsTM;


    initialiseObjects();

    BVH bvh = BVH();
    bvh.constructBVHST(sceneObjectsList);
    rootNode = bvh.getBVHNodes()[0];
    emissiveObjects = systemManager->getSceneObjectManager()->getEmmisiveObjects();

    // render loop
    while (systemManager->getIsRunning()) {
        auto frameStartTime = std::chrono::high_resolution_clock::now();

        numThreads = config.threads > 0 ? config.threads : std::thread::hardware_concurrency();
        int segments = std::round(std::sqrt(numThreads));

        if (UI::camUpdate || camera->getCamMoved() || !UI::accumulateRays) {
            if (UI::camUpdate) {
                config.fOV = UI::fOV;
                config.exposure = UI::exposure;
                config.DepthOfField = UI::depthOfField;
                config.focalDistance = UI::focalDistance;
                config.spatialSampling = UI::spatialSampling;
                config.temporalSampling = UI::temporalSampling;
                config.minBounces = UI::minBounces;
                config.maxBounces = UI::maxBounces;
                emissiveObjects = systemManager->getSceneObjectManager()->getEmmisiveObjects();
                camera->reInitilize(); // fov changes
            }

            if (UI::sceneUpdate) {
                UI::sceneUpdate = false;
                bvh.constructBVHST(sceneObjectsList);
                rootNode = bvh.getBVHNodes()[0];
                emissiveObjects = systemManager->getSceneObjectManager()->getEmmisiveObjects();
            }

            if (UI::resUpdate) {
                initialiseObjects(); // change buffer size
                denoiser->resize(internalResX * internalResY);
                UI::resUpdate = false;
                UI::resizeBuffer = true;
            }

            // reset colour buffer
            for (int i = 0; i < internalResX * internalResY; i++) {
                lum[i] = Vector3(0, 0, 0);
                hdr[i] = Vector3(0, 0, 0);
            }

            config.ReSTIR = UI::ReSTIR;
            config.ReSTIRGI = UI::ReSTIRGI;
            iterations = 0;
            UI::accumulatedRays = iterations * config.raysPerPixel;
            UI::camUpdate = false;
            camera->getCamMoved() = false;
        }

        if (UI::upscalingUpdate) {
            config.upScale = UI::upscale;
            updateUpscaling();
            UI::upscalingUpdate = false;
        }

        std::chrono::duration<long long, std::ratio<1, 1000> > durationTimeRays;

        Camera cameraCopy = *camera; // dereference camera and copy
        bool sky = config.sky; // copy bool

        for (int currentRay = 1; currentRay <= config.raysPerPixel; currentRay++) {
            ////////////////////////////////////////////////////////////////////////
            // Path tracing - Multiple Importance Sampling + Direct Illumination
            ////////////////////////////////////////////////////////////////////////


            auto startTimeRays = std::chrono::high_resolution_clock::now();

            // Calculate number of screen segments
            int tileSize = config.tileSize;
            int segmentsX = (internalResX + tileSize - 1) / tileSize; // Ceiling division
            int segmentsY = (internalResY + tileSize - 1) / tileSize;
            int totalSegments = segmentsX * segmentsY;
            std::atomic<int> nextSegment(0);

            // Create a worker for each segment
            auto workerPathTracer = [this, &nextSegment, totalSegments, segmentsX, tileSize, &cameraCopy, currentRay, sky, &mutex]() {
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
                    traceRay(cameraCopy, startX, endX, startY, endY, iterations, currentRay, sky, mutex);
                }
            };

            // Launch worker threads
            std::vector<std::thread> threadsPT;
            for (int i = 0; i < numThreads; ++i) {
                threadsPT.emplace_back(workerPathTracer);
            }
            // Block until all threads are finished

            for (std::thread &thread: threadsPT) {
                thread.join();
            }
            threadsPT.clear();

            ////////////////////////////////////////////////////////////////////////
            // Path tracing - ReSTIR SpatioTemporal Resampling
            ////////////////////////////////////////////////////////////////////////

            // Create a worker for each segment
            nextSegment = 0;
            auto workerReSTIR = [this, &nextSegment, totalSegments, segmentsX, tileSize, currentRay, &mutex]() {
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
                    restirSpatioTemporal(startX, endX, startY, endY, iterations, currentRay, mutex);
                }
            };

            // Launch worker threads
            std::vector<std::thread> threadsReSTIR;
            for (int i = 0; i < numThreads; ++i) {
                threadsReSTIR.emplace_back(workerReSTIR);
            }
            // Block until all threads are finished

            for (std::thread &thread: threadsReSTIR) {
                thread.join();
            }
            threadsReSTIR.clear();

            //restirSpatioTemporal(0, internalResX, 0, internalResY, iterations, currentRay, mutex);

            durationTimeRays = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeRays);
            UI::pathTracingTime = std::chrono::duration<float>(durationTimeRays).count() * 1000;
        }

        ////////////////////////////////////
        // Denoising
        ////////////////////////////////////
        auto startTimeDN = std::chrono::high_resolution_clock::now();

        config.denoise = UI::denoise;
        config.denoiseIterations = UI::denoiseIterations;

        if (config.denoise) {
            denoiser->launchDenoiseThreads(lum, normalBuffer, depthBuffer, albedoBuffer, emissionBuffer, config.denoiseIterations, internalResX, internalResY,
                                           segments);
        }

        auto durationTimeDN = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeDN);
        UI::denoisingTime = std::chrono::duration<float>(durationTimeDN).count() * 1000;

        ////////////////////////////////////
        // Tone mapping
        ////////////////////////////////////

        auto startTimeTM = std::chrono::high_resolution_clock::now();

        toneMapper->launchToneMappingThreads(lum, RGBBuffer, maxLuminance, segments, resX, resY, internalResX, internalResY, upScale, numThreads);

        systemManager->updateRGBBuffer(RGBBuffer); // push latest screen buffer

        //-----------------------

        auto durationTimeTM = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeTM);
        auto finalFrameTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - frameStartTime);
        UI::toneMappingTime = std::chrono::duration<float>(durationTimeTM).count() * 1000;
        UI::frameTime = std::chrono::duration<float>(finalFrameTime).count() * 1000;

        iterations++;
        UI::accumulatedRays = iterations * config.raysPerPixel;

        float frameTimeSec = std::chrono::duration<float>(durationTimeRays).count();
        UI::RaysPerSecond = (config.raysPerPixel * internalResX * internalResY) / (frameTimeSec);
    }
}

void CPUPT::traceRay(Camera camera, int xstart, int xend, int ystart, int yend, int its, int currentRay, bool sky, std::mutex &mutex) const {
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);

    for (int y = ystart; y <= yend; y++) {
        for (int x = xstart; x <= xend; x++) {
            // -------------------------------------------------------------
            // Primary (camera) Ray setup
            // -------------------------------------------------------------
            Ray ray;
            ray.reset();
            // Camera (primary) ray origin:
            ray.getOrigin().set(camera.getPos());
            ray.getPos().set(camera.getPos());

            // Jitter the pixel position for MSAA effect
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

            for (int currentBounce = 0; currentBounce <= config.maxBounces; currentBounce++) {
                // Intersect scene using BVH
                BVHNode::BVHResult leafNode = rootNode->searchBVHTreeScene(ray);
                if (leafNode.node == nullptr || !leafNode.node->getLeaf()) {
                    // Ray intersects nothing, break
                    if (sky) {
                        Vector3 skyColour(0.247f, 0.247f, 0.247f);
                        finalColour = finalColour + throughput * skyColour * 0.25;
                    }
                    if (currentBounce == 0) reservoirReSTIR[y * internalResX + x] = Reservoir{}; // reset reservoir
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
                Material *mat = hitObject->getMaterial();

                // Fill denoising buffers
                if (currentBounce == 0 && config.denoise == true) {
                    depthBuffer[y * internalResX + x] = leafNode.close;
                    albedoBuffer[y * internalResX + x] = mat->colour;
                    emissionBuffer[y * internalResX + x] = mat->emission;
                }

                normalBuffer[y * internalResX + x] = ray.getNormal();
                // -------------------------------------------------------------
                // ReSTIR Direct Illumination (first bounce only)
                // -------------------------------------------------------------
                if (currentBounce == 0 && config.ReSTIR) restirDirectLighting(ray, hitObject, x, y);

                // -------------------------------------------------------------
                // Multiple Importance Sampling
                // -------------------------------------------------------------
                // 1) Sample new direction: reflection or refraction and compute BRDF and PDF
                float randomSample = dist(rng);
                float p_specular = mat->metallic;
                float p_transmission = mat->transmission * (1.0f - mat->metallic);
                float p_diffuse = 1.0f - (p_specular + p_transmission);
                // probabilites should add up to 1

                Vector3 wo = -ray.getDir(); // outgoing direction
                Vector3 wi;

                Vector3 n = ray.getNormal();
                Vector3 newThroughput;

                // specular caused by IOR
                float cosTheta = std::abs(ray.getDir().dot(ray.getNormal()));
                float R0 = surfaceIntegrator->fresnelSchlickRefraction(cosTheta, mat->IOR); // reflection portion
                float randomSample2 = dist(rng); // a second sample
                // ----------------------

                if (ray.getInternal()) {
                    // Refraction
                    // Continue refraction from previous bounce
                    wi = directionSampler->RefractionDirection(ray, *hitObject);
                    newThroughput = surfaceIntegrator->computeThroughput(wo, wi, mat, n, R0, refreaction, true);
                } else {
                    if (randomSample <= p_specular) {
                        // Specular (Metallic)
                        wi = directionSampler->SpecularDirection(ray, *hitObject, false);
                        newThroughput = surfaceIntegrator->computeThroughput(wo, wi, mat, n, R0, metallic, false);
                    } else if (randomSample <= p_specular + p_transmission) {
                        // Blend in the possibility of refraction based on (transmission * (1 - metallic))
                        if (randomSample2 < R0) {
                            // Specular (Glass)
                            wi = directionSampler->SpecularDirection(ray, *hitObject, false);
                            newThroughput = surfaceIntegrator->computeThroughput(wo, wi, mat, n, R0, specularFresnel, false);
                        } else {
                            // Refraction
                            wi = directionSampler->RefractionDirection(ray, *hitObject);
                            newThroughput = surfaceIntegrator->computeThroughput(wo, wi, mat, n, R0, refreaction, false);
                        }
                    } else if (randomSample <= p_specular + p_transmission + p_diffuse) {
                        if (randomSample2 < R0) {
                            // Specular Diffuse
                            wi = directionSampler->SpecularDirection(ray, *hitObject, false);
                            newThroughput = surfaceIntegrator->computeThroughput(wo, wi, mat, n, R0, specularFresnel, false);
                        } else {
                            // Dielectric reflection
                            wi = directionSampler->DiffuseDirection(ray, *hitObject, false); // sample direction
                            newThroughput = surfaceIntegrator->computeThroughput(wo, wi, mat, n, R0, diffuse, false);
                        }
                    } else {
                        std::cout << "Probabilities dont add up: " << randomSample << std::endl;
                        break;
                    }
                }

                // 2) Update throughput with BRDF and PDF
                throughput = throughput * newThroughput;

                // 3) Update colour
                finalColour = finalColour + throughput * (mat->colour * mat->emission);
                finalColour.sanitize();

                // -------------------------------------------------------------
                // Russian roulete termination
                // -------------------------------------------------------------

                float RR = std::min(std::max(throughput.maxComponent(), 0.1f), 1.0f);

                // Ensure minimum number of bounces completed
                if (currentBounce > config.minBounces) {
                    if (dist(rng) > RR) {
                        // Add the remaining contribution before termination
                        finalColour = finalColour + throughput * (mat->colour * mat->emission);
                        break; // Terminate the ray path early
                    }
                    // Scale the throughput to maintain an unbiased estimator
                    throughput = throughput / RR;
                }

                // Prepare for next bounce
                ray.getDir().set(wi);
                ray.updateOrigin(0.01);
            } // end for bounceDepth

            // -------------------------------------------------------------
            // Ray terminated. Accumulate into the buffer.
            // -------------------------------------------------------------
            hdr[y * internalResX + x] = hdr[y * internalResX + x] + finalColour;
            lum[y * internalResX + x] = hdr[y * internalResX + x] + finalColour;
        } // end x
    } // end y
}

void CPUPT::restirDirectLighting(Ray &ray, SceneObject *hitObject, int x, int y) const {
    Reservoir reservoir = {};

    if (emissiveObjects.size() == 0) {
        reservoirReSTIR[y * internalResX + x] = reservoir;
        return;
    }

    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    std::uniform_int_distribution<int> lightDist(0, emissiveObjects.size() - 1);
    Material *hitMat = hitObject->getMaterial();
    const int M = config.lightSamples;

    Vector3 rayPos = ray.getPos();
    Vector3 n = ray.getNormal();
    Vector3 wo = -ray.getDir();

    for (int i = 0; i < M; i++) {
        reservoir.sampleCount++;
        int lightIndex = lightDist(rng);
        SceneObject *light = emissiveObjects[lightIndex];
        const Material *lightMat = light->getMaterial();
        Vector3 lightPoint = light->samplePoint(dist(rng), dist(rng));
        Vector3 wi = lightPoint - rayPos;
        wi.normalise();
        float distToLight = (lightPoint - rayPos).length();
        float cos_theta = n.dot(wi);
        float importance = 0.0f;
        float PDF_light = (1.0f / emissiveObjects.size()) * (1.0f / light->getArea());
        Vector3 lightEmission = lightMat->colour * lightMat->emission;
        if (cos_theta > 0.0f) {
            Vector3 BRDF_light = surfaceIntegrator->evaluateBRDF(wo, wi, hitMat, n);
            Vector3 lightNormal = light->getNormal(lightPoint); // Assume this method exists
            float cos_theta_y = std::max(0.0f, -lightNormal.dot(wi));
            Vector3 contrib = lightEmission * cos_theta_y * BRDF_light * (cos_theta / (distToLight * distToLight));
            importance = Vector3::luminance(contrib) / PDF_light;
        }
        reservoir.weightSum += importance;
        if (dist(rng) * reservoir.weightSum < importance) {
            reservoir.candidatePosition = lightPoint;
            reservoir.candidateEmission = lightEmission;
            reservoir.PDF = PDF_light;
            reservoir.distToLight = distToLight;
            reservoir.hitMat = hitMat;
            reservoir.candidateNormal = light->getNormal(lightPoint);
        }
    }
    if (reservoir.weightSum == 0) {
        reservoirReSTIR[y * internalResX + x] = Reservoir{};
        return; // early exist
    }

    // Shadow test
    Vector3 wi = (reservoir.candidatePosition - rayPos);
    Vector3::normalise(wi);
    Ray shadowRay(rayPos + wi * 0.01f, wi);
    BVHNode::BVHResult shadowResult = rootNode->searchBVHTreeScene(shadowRay);
    if (shadowResult.node == nullptr || !shadowResult.node->getLeaf() ||
        !(shadowResult.close < reservoir.distToLight + 0.03f && shadowResult.close > reservoir.distToLight - 0.03f)) {
        reservoirReSTIR[y * internalResX + x] = Reservoir{};
        return;
    }

    reservoir.rayPos = ray.getPos();
    reservoir.n = ray.getNormal();
    reservoir.wo = -ray.getDir();
    reservoirReSTIR[y * internalResX + x] = reservoir;
}

void CPUPT::restirSpatioTemporal(int xstart, int xend, int ystart, int yend, int its, int currentRay, std::mutex &mutex) const {
    // iterate over each pixel

    for (int y = ystart; y <= yend; y++) {
        for (int x = xstart; x <= xend; x++) {
            if (config.ReSTIR) {
                Vector3 totalContribution{0.0f};
                Reservoir currentReservoir = reservoirReSTIR[y * internalResX + x];
                //if (currentReservoir.weightSum == 0) continue;

                int range = config.spatialSampling;
                float samples = 0;
                // select neighbouring resevoirs around that pixel
                for (int j = y - range; j <= y + range; j++) {
                    if (j < 0 || j >= internalResY) continue;
                    for (int i = x - range; i <= x + range; i++) {
                        if (i < 0 || i >= internalResX) continue;
                        Reservoir sampleReservoir = reservoirReSTIR[j * internalResX + i];
                        float similarity = std::max(0.0f, Vector3::dot(normalBuffer[j * internalResX + i], normalBuffer[y * internalResX + x]));

                        if (similarity < 0.5) continue;
                        samples++;
                        if (sampleReservoir.weightSum == 0) continue;

                        const int M = config.lightSamples;
                        Material *hitMat = sampleReservoir.hitMat;
                        Vector3 rayPos = sampleReservoir.rayPos;
                        Vector3 n = sampleReservoir.n;
                        Vector3 wo = sampleReservoir.wo;
                        Vector3 wi = (sampleReservoir.candidatePosition - rayPos);
                        wi.normalise();

                        float cos_theta = std::max(0.0f, n.dot(wi));
                        Vector3 BRDF = surfaceIntegrator->evaluateBRDF(wo, wi, hitMat, n);
                        Vector3 numerator = BRDF * sampleReservoir.candidateEmission * (
                                                cos_theta / (sampleReservoir.distToLight * sampleReservoir.distToLight));
                        float w_selected = Vector3::luminance(numerator) / sampleReservoir.PDF;
                        Vector3 contribution = (numerator / sampleReservoir.PDF) * (sampleReservoir.weightSum / (M * w_selected));

                        totalContribution += contribution * similarity;
                    }
                }

                if (samples != 0) {
                    // Update un-normalised buffer
                    hdr[y * internalResX + x] = hdr[y * internalResX + x] + totalContribution / samples;
                }
            } // end ReSTIR

            // normalise progress buffer
            lum[y * internalResX + x] = hdr[y * internalResX + x] / (static_cast<float>(currentRay) + static_cast<float>(its) * static_cast<float>(config.raysPerPixel));
        } // end x
    } // end y
}


void CPUPT::initialiseObjects() {
    resX = config.resX;
    resY = config.resY;

    upScale = config.upScale;

    internalResX = resX / upScale;
    internalResY = resY / upScale;
    int res = internalResX * internalResY;

    aspectRatio = static_cast<float>(resX) / resY;

    // Pixel buffer
    RGBBuffer = new uint8_t[resX * resY * 3];

    // Raw pixel data
    lum.resize(res, Vector3(0.0f, 0.0f, 0.0f));
    hdr.resize(res, Vector3(0.0f, 0.0f, 0.0f));

    // ReSTIR and ReSTIRGI
    reservoirReSTIR.resize(res, Reservoir{});
    reservoirReSTIRGI.resize(res, ReservoirGI{});

    // Denoising
    normalBuffer.resize(res, 0.0f);
    depthBuffer.resize(res, 0.0f);
    albedoBuffer.resize(res, 0.0f);
    emissionBuffer.resize(res, 0.0f);

    denoiser->resize(res);
}

void CPUPT::updateUpscaling() {
    upScale = config.upScale;

    internalResX = config.resX / config.upScale;
    internalResY = resY / config.upScale;

    int res = internalResX * internalResY;

    // Raw pixel data
    lum.resize(res, Vector3(0.0f, 0.0f, 0.0f));
    hdr.resize(res, Vector3(0.0f, 0.0f, 0.0f));

    // ReSTIR and ReSTIRGI
    reservoirReSTIR.resize(res, Reservoir{});
    reservoirReSTIRGI.resize(res, ReservoirGI{});

    // Denoising
    normalBuffer.resize(res, 0.0f);
    depthBuffer.resize(res, 0.0f);
    albedoBuffer.resize(res, 0.0f);
    emissionBuffer.resize(res, 0.0f);
}

void CPUPT::deleteObjects() {
    delete[] RGBBuffer;
    delete directionSampler;
    delete surfaceIntegrator;
    delete denoiser;
    delete toneMapper;
}

SceneObject *CPUPT::getClickedObject(int screenX, int screenY) {
    int x = screenX / upScale;
    int y = screenY / upScale;

    // Ray cast to find clicked object
    Ray ray;
    ray.reset();
    ray.setDebugTrue(); // for debug
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

    if (leafNode.node == nullptr || !leafNode.node->getLeaf()) {
        return nullptr;
    }

    return leafNode.node->getSceneObject();
}

void CPUPT::debugRay(int screenX, int screenY) {
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);

    int x = screenX / upScale;
    int y = screenY / upScale;

    // Ray cast to find clicked object
    Ray ray;
    ray.reset();
    ray.setDebugTrue();
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

    // Start loop
    Vector3 finalColour(0.0f, 0.0f, 0.0f);
    Vector3 throughput(1.0f, 1.0f, 1.0f); // Running throughput
    std::cout << "----------------------------" << std::endl;
    std::cout << "New Ray:" << std::endl;

    ray.setInternal(false);
    for (int currentBounce = 0; currentBounce <= config.maxBounces; currentBounce++) {
        std::cout << "Bounce: " << currentBounce << std::endl;

        std::cout << "Ray direction";
        ray.getDir().print();

        // Intersect scene using BVH
        BVHNode::BVHResult leafNode = rootNode->searchBVHTreeScene(ray);
        if (leafNode.node == nullptr || !leafNode.node->getLeaf()) {
            // Ray intersects nothing, break
            std::cout << "No intersection" << std::endl;
            break;
        }

        // Move the ray to the exact hit point
        SceneObject *hitObject = leafNode.node->getSceneObject();
        std::cout << "Intersection with: ";
        hitObject->printType();
        std::cout << "Emission: " << hitObject->getMaterial()->emission << std::endl;;
        if (!ray.getInternal()) {
            // outside of an object - march to entry
            ray.march(leafNode.close);
            std::cout << "Ray external" << std::endl;
        } else {
            // inside of an object - march to far side
            ray.march(leafNode.far);
            std::cout << "Ray internal" << std::endl;
        }
        std::cout << "New ray position";
        ray.getPos().print();

        hitObject->getNormal(ray); // sets ray.getNormal()
        std::cout << "Object Normal: (external facing)";
        ray.getNormal().print();
        ray.getOrigin().set(ray.getPos()); // Set new ray origin
        ray.setHitObject(hitObject);

        // Grab the material
        Material *mat = hitObject->getMaterial();

        std::cout << "ReSTIR" << std::endl;
        // Compute direct lighting with ReSTIR at first bounce
        if (currentBounce == 0 && config.ReSTIR) {
            /*restirDirectLighting(ray, hitObject, x, y);
            //Vector3 directLighting = restirDirectLighting(ray, hitObject, x, y);

            Vector3 directLighting = restirSpatioTemporal(ray, hitObject, x, y);
            std::cout<<"Direct lighing: ";
            directLighting.print();
            finalColour = finalColour + directLighting;*/
        }

        // 1) Sample new direction: reflection or refraction and compute BRDF and PDF
        float randomSample = dist(rng);
        float p_specular = mat->metallic;
        float p_transmission = mat->transmission * (1.0f - mat->metallic);
        float p_diffuse = 1.0f - (p_specular + p_transmission);
        // probabilites should add up to 1

        Vector3 wo = -ray.getDir(); // outgoing direction
        Vector3 wi;

        Vector3 n = ray.getNormal();
        Vector3 newThroughput;

        // specular caused by IOR
        float cosTheta = std::abs(ray.getDir().dot(ray.getNormal()));
        float R0 = surfaceIntegrator->fresnelSchlickRefraction(cosTheta, mat->IOR); // reflection portion
        float randomSample2 = dist(rng); // a second sample
        // ----------------------

        if (ray.getInternal()) {
            // Refraction
            // Continue refraction from previous bounce
            std::cout << "Continuing previous refraction" << std::endl;
            wi = directionSampler->RefractionDirection(ray, *hitObject);
            newThroughput = surfaceIntegrator->computeThroughput(wo, wi, mat, n, R0, refreaction, true);
        } else {
            if (randomSample <= p_specular) {
                // Specular (Metallic)
                std::cout << "Sampling specular (metallic)" << std::endl;
                wi = directionSampler->SpecularDirection(ray, *hitObject, false);
                newThroughput = surfaceIntegrator->computeThroughput(wo, wi, mat, n, R0, metallic, false);
            } else if (randomSample <= p_specular + p_transmission) {
                // Blend in the possibility of refraction based on (transmission * (1 - metallic))
                if (randomSample2 < R0) {
                    // Specular (Glass)
                    std::cout << "Sampling specular (fresnel)" << std::endl;
                    wi = directionSampler->SpecularDirection(ray, *hitObject, false);
                    newThroughput = surfaceIntegrator->computeThroughput(wo, wi, mat, n, R0, specularFresnel, false);
                } else {
                    // Refraction
                    std::cout << "Sampling refraction" << std::endl;
                    wi = directionSampler->RefractionDirection(ray, *hitObject);
                    newThroughput = surfaceIntegrator->computeThroughput(wo, wi, mat, n, R0, refreaction, false);
                }
            } else if (randomSample <= p_specular + p_transmission + p_diffuse) {
                if (randomSample2 < R0) {
                    // Specular Diffuse
                    std::cout << "Sampling specular (fresnel)" << std::endl;
                    wi = directionSampler->SpecularDirection(ray, *hitObject, false);
                    newThroughput = surfaceIntegrator->computeThroughput(wo, wi, mat, n, R0, specularFresnel, false);
                } else {
                    // Dielectric reflection
                    std::cout << "Sampling diffuse (fresnel)" << std::endl;
                    wi = directionSampler->DiffuseDirection(ray, *hitObject, false); // sample direction
                    newThroughput = surfaceIntegrator->computeThroughput(wo, wi, mat, n, R0, diffuse, false);
                }
            } else {
                std::cout << "Probabilities dont add up: " << randomSample << std::endl;
                break;
            }
        }
        std::cout << "New throughput";
        newThroughput.print();


        // 2) Add emission *through* the throughput
        finalColour = finalColour + throughput * (mat->colour * mat->emission);

        // 3) Update throughput with BRDF and PDF
        throughput = throughput * newThroughput;
        std::cout << "Updated throughput";
        throughput.print();

        // -------------------------------------------------------------
        // Russian roulete termination
        // -------------------------------------------------------------

        float RR = std::min(std::max(throughput.maxComponent(), 0.1f), 1.0f);

        // Ensure minimum number of bounces completed
        if (currentBounce > config.minBounces) {
            if (dist(rng) > RR) {
                // Add the remaining contribution before termination
                finalColour = finalColour + throughput * (mat->colour * mat->emission);
                break; // Terminate the ray path early
            }
            // Scale the throughput to maintain an unbiased estimator
            throughput = throughput / RR;
        }

        // Prepare for next bounce
        ray.getDir().set(wi);
        ray.updateOrigin(0.01);
        std::cout << "----------------------------" << std::endl;
    } // end for bounceDepth
}

void CPUPT::debugPixelInfo(int screenX, int screenY) {
    std::cout << "X: " << screenX << ", Y: " << screenY << std::endl;
    std::cout << "HDR: ";
    hdr[screenY * internalResX + screenX].print();
    std::cout << "LUM: ";
    lum[screenY * internalResX + screenX].print();
}

std::pair<int, int> CPUPT::threadSegments(float start, float end, int &numThreads, int step) {
    int res = end - start;
    int pixelsPerThread = res / numThreads; // number of pixels per thread
    int remainder = res % numThreads; // remaining pixels after division

    int startPixel = step * pixelsPerThread + std::min(step, remainder);
    int endPixel = (step + 1) * pixelsPerThread + std::min(step + 1, remainder) - 1;

    return std::make_pair(startPixel, endPixel);
}
