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
    materialManager = systemManager->getMaterialManager();
    directionSampler = new DirectionSampler();
    surfaceIntegrator = new SurfaceIntegrator(materialManager);
    denoiser = new Denoiser();
    toneMapper = new ToneMapper();

    camera = systemManager->getSceneObjectManager()->getCamera();

    dist = std::uniform_real_distribution<float>(0.0f, 1.0f);
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
    spatialDist = std::uniform_int_distribution<int>(-config.sampleRadius, config.sampleRadius);
    lightDist = std::uniform_int_distribution<int>(0, emissiveObjects.size() - 1);

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

            config.BSDF = UI::BSDF;
            config.NEE = UI::NEE;
            config.ReSTIR = UI::ReSTIR;
            config.ReSTIRGI = UI::ReSTIRGI;
            config.unbiased = UI::unbiased;
            config.candidateSamples = UI::candidateSamples;
            config.spatialSamplesK = UI::spatialSamplesK;
            config.NEEsamples = UI::NEEsamples;
            UI::accumulatedRays = iterations * config.raysPerPixel;
            UI::camUpdate = false;
            camera->getCamMoved() = false;
            iterations = 0;
            dist = std::uniform_real_distribution<float>(0.0f, 1.0f);
            spatialDist = std::uniform_int_distribution<int>(-config.sampleRadius, config.sampleRadius);
            lightDist = std::uniform_int_distribution<int>(0, emissiveObjects.size() - 1);
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
                    traceRay(cameraCopy, startX, endX, startY, endY, sky, mutex);
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

void CPUPT::traceRay(Camera camera, int xstart, int xend, int ystart, int yend, bool sky, std::mutex &mutex) const {
    for (int y = ystart; y <= yend; y++) {
        for (int x = xstart; x <= xend; x++) {
            // -------------------------------------------------------------
            // Primary (camera) Ray setup
            // -------------------------------------------------------------
            Ray ray;
            ray.reset();
            // Camera (primary) ray origin:
            ray.getPos().set(camera.getPos());
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
                ray.getPos().set(camera.getPos() + lensOffset);
                ray.getPos().set(camera.getPos() + lensOffset);
                ray.getDir().set(focalPoint - ray.getPos());
                ray.getDir().normalise();
            }

            // -------------------------------------------------------------
            // Path Tracing Loop
            // -------------------------------------------------------------
            Vector3 finalColour(0.0f, 0.0f, 0.0f);
            Vector3 throughput(1.0f, 1.0f, 1.0f); // Running throughput

            for (int currentBounce = 0; currentBounce <= config.maxBounces; currentBounce++) {
                // Intersect scene using BVH
                BVHNode::BVHResult leafNode = rootNode->searchBVHTreeScene(ray);
                ray.setTriangle(leafNode.triangle);
                ray.getBCoords().set(leafNode.bCoords);
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
                //ray.getOrigin().set(ray.getPos()); // Set new ray origin
                ray.setHitObject(hitObject);

                // Compute material properties at the hitpoint (mesh objects)
                Material *sampledMat = materialManager->sampleMatFromHitPoint(ray, hitObject->getMaterial(ray), hitObject);

                if (currentBounce == 0) {
                    // primary ray only
                    // ReSTIR
                    depthBuffer[y * internalResX + x] = leafNode.close;
                    normalBuffer[y * internalResX + x] = ray.getNormal();
                    // Denoising
                    albedoBuffer[y * internalResX + x] = sampledMat->colour;
                    emissionBuffer[y * internalResX + x] = sampledMat->emission;
                }

                // -------------------------------------------------------------
                // ReSTIR Direct Illumination (first bounce only)
                // -------------------------------------------------------------

                if (currentBounce == 0 && config.ReSTIR) {
                    delete reservoirReSTIR[y * internalResX + x].hitMat;
                    Material *ReSTIRsampledMat = materialManager->sampleMatFromHitPoint(ray, hitObject->getMaterial(ray), hitObject);
                    restirDirectLighting(ray, ReSTIRsampledMat, x, y);
                }

                // -------------------------------------------------------------
                // Multiple Importance Sampling
                // -------------------------------------------------------------
                // 1) Sample new direction: reflection or refraction and compute BRDF and PDF
                float randomSample = dist(rng);
                float p_specular = sampledMat->metallic;
                float p_transmission = sampledMat->transmission * (1.0f - sampledMat->metallic);
                float p_diffuse = 1.0f - (p_specular + p_transmission);
                // probabilites should add up to 1

                Vector3 wo = -ray.getDir(); // outgoing direction
                Vector3 wi;

                Vector3 n = ray.getNormal();
                Vector3 newThroughput;

                // specular caused by IOR
                Vector3 h = Vector3::halfVector(wo, wi);
                float cosTheta_H = std::abs(Vector3::dot(wo, h));
                float R0 = surfaceIntegrator->fresnelSchlickIOR(cosTheta_H, sampledMat->IOR); // reflection portion
                float randomSample2 = dist(rng); // a second sample
                // ----------------------

                if (ray.getInternal()) {
                    // Refraction
                    // Continue refraction from previous bounce
                    wi = directionSampler->RefractionDirection(ray, *hitObject);
                    newThroughput = surfaceIntegrator->computeThroughput(wo, wi, sampledMat, n, R0, refrecation, true);
                } else {
                    if (randomSample <= p_specular) {
                        // Specular (Metallic)
                        wi = directionSampler->SpecularDirection(ray, *hitObject, false);
                        newThroughput = surfaceIntegrator->computeThroughput(wo, wi, sampledMat, n, R0, metallic, false);
                    } else if (randomSample <= p_specular + p_transmission) {
                        // Blend in the possibility of refraction based on (transmission * (1 - metallic))
                        if (randomSample2 < R0) {
                            // Specular (Glass)
                            wi = directionSampler->SpecularDirection(ray, *hitObject, false);
                            newThroughput = surfaceIntegrator->computeThroughput(wo, wi, sampledMat, n, R0, specularFresnel, false);
                        } else {
                            // Refraction
                            wi = directionSampler->RefractionDirection(ray, *hitObject);
                            newThroughput = surfaceIntegrator->computeThroughput(wo, wi, sampledMat, n, R0, refrecation, false);
                        }
                    } else if (randomSample <= p_specular + p_transmission + p_diffuse) {
                        if (randomSample2 < R0) {
                            // Specular Diffuse
                            wi = directionSampler->SpecularDirection(ray, *hitObject, false);
                            newThroughput = surfaceIntegrator->computeThroughput(wo, wi, sampledMat, n, R0, specularFresnel, false);
                        } else {
                            // Dielectric reflection
                            wi = directionSampler->DiffuseDirection(ray, *hitObject, false); // sample direction
                            newThroughput = surfaceIntegrator->computeThroughput(wo, wi, sampledMat, n, R0, diffuse, false);
                        }
                    } else {
                        std::cout << "Probabilities dont add up: " << randomSample << std::endl;
                        break;
                    }
                }
                /*if (newThroughput.x > 1.0f || newThroughput.y > 1.0f || newThroughput.z > 1.0f) {
                    std::cout << "Large throughput at bounce " << currentBounce<<std::endl;
                    std::cout << "Original throughput: ";
                    throughput.print();
                    std::cout << "newThroughput: ";
                    newThroughput.print();
                    std::cout << "throughput * newThroughput: ";
                    (throughput * newThroughput).print();
                    std::cout<<"---------------"<<std::endl;
                }*/
                // 2) Update throughput with BRDF and PDF
                throughput = throughput * newThroughput;

                // 3) Update colour
                if (config.BSDF) finalColour = finalColour + throughput * (sampledMat->colour * sampledMat->emission);

                // -------------------------------------------------------------
                // NEE Direct Lighting (every bounce, except ReSTIR bounces)
                // -------------------------------------------------------------

                //if (config.NEE && !(config.ReSTIR && currentBounce == 0)) finalColour += throughput * directLightingNEE(ray, sampledMat);
                if (config.NEE) finalColour += throughput * directLightingNEE(ray, sampledMat);

                delete sampledMat;
                // -------------------------------------------------------------
                // Russian roulete termination
                // -------------------------------------------------------------

                float RR = std::min(std::max(throughput.maxComponent(), 0.1f), 1.0f);

                // Ensure minimum number of bounces completed
                if (currentBounce > config.minBounces) {
                    if (dist(rng) > RR) break; // Terminate the ray path early
                    // Scale the throughput to maintain an unbiased estimator
                    throughput = throughput / RR;
                }

                // Prepare for next bounce
                ray.getDir().set(wi);
                ray.march(0.01);
            } // end for bounceDepth

            // -------------------------------------------------------------
            // Ray terminated. Accumulate into the buffer.
            // -------------------------------------------------------------
            hdr[y * internalResX + x] += finalColour;
        } // end x
    } // end y
}

Vector3 CPUPT::directLightingNEE(Ray &ray, Material *sampledMat) const {

    if (emissiveObjects.size() == 0) return {0};
    Reservoir reservoir = {};

    Vector3 rayPos = ray.getPos();
    Vector3 n = ray.getNormal();
    Vector3 wo = -ray.getDir();
    reservoir.rayPos = rayPos;
    reservoir.n = n;
    reservoir.wo = wo;
    reservoir.hitMat = sampledMat;

    // M = config.lightSamples

    for (int i = 0; i < config.NEEsamples; i++) {
        reservoir.sampleCount++;

        SceneObject *light = emissiveObjects[lightDist(rng)];
        Material *lightMat = light->getMaterial(); // need to adjust to take into account the triangles specific material
        Vector3 lightPoint = light->samplePoint(dist(rng), dist(rng));
        Vector3 wi = rayPos - lightPoint;
        float distToLight = wi.length();
        wi.normalise();
        float cosTheta_q = std::max(0.0f, n.dot(-wi));
        float cosTheta_x = std::max(0.0f, light->getNormal(lightPoint).dot(wi));
        if (cosTheta_q == 0 || cosTheta_x == 0) continue; // light point is facing away or behind hit point
        // = hitBRDF * emission * (distance^2 * cosTheta)
        // Pq(x) = hitPoint BRDF

        Vector3 BRDF_x = surfaceIntegrator->evaluateBRDF(wo, wi, sampledMat, n);
        // G(x) = cosTheta(x) * (cosTheta(q) / (x - q)^2
        float G = (cosTheta_x * cosTheta_q) / (distToLight * distToLight);
        // Le(x) = emission of light point
        Vector3 Le = lightMat->colour * lightMat->emission;

        // PDF of light point = (1 / lightArea) * (1 / numLights)
        float lightArea = light->getArea();
        float lightPDF = (1.0f / lightArea) * (1.0f / emissiveObjects.size());
        // (BRDF(q,x) * Le(x) * G(q,x)) / (1 / surfaceArea)
        Vector3 contribution = BRDF_x * Le * G; // unshadowed path contribution

        float targetPDF = Vector3::length(contribution);
        float candidateWeight = targetPDF / lightPDF; // w_i = p̂(x_i)/p(x_i)

        reservoir.weightSum += candidateWeight;
        if (dist(rng) < candidateWeight / reservoir.weightSum) {
            reservoir.candidatePosition = lightPoint;
            reservoir.distToLight = distToLight;
            // extra things im storing for now
            reservoir.targetPDF = targetPDF;
            reservoir.candidatePDF = lightPDF;
            reservoir.candidateEmission = light->getMaterial()->emission;
            reservoir.candidateNormal = light->getNormal(lightPoint);
            reservoir.lightMat = lightMat;
            reservoir.lightArea = lightArea;
            reservoir.empty = false;
        }
    }

    // Compute final contribution
    if (reservoir.empty || reservoir.weightSum == 0) {
        return {0};
    }


    // Shadow ray test
    Vector3 wi = reservoir.candidatePosition - rayPos;
    float distToLight = Vector3::length(wi);
    if (distToLight < 0.01f) return {0};
    Vector3::normalise(wi);

    // Shadow test
    Ray shadowRay(reservoir.rayPos + wi * 0.01f, wi);
    distToLight -= 0.0f;
    BVHNode::BVHResult shadowResult = rootNode->searchBVHTreeScene(shadowRay);
    if (shadowResult.node == nullptr || !shadowResult.node->getLeaf() || shadowResult.node->getSceneObject()->getMaterial()->emission == 0 ||
        !(shadowResult.close < distToLight + 0.05f && shadowResult.close > distToLight - 0.05f)) {
        // occulsion
        return {0};
    }

    float cosTheta_q = std::max(0.0f, reservoir.n.dot(wi));
    float cosTheta_x = std::max(0.0f, reservoir.candidateNormal.dot(-wi));

    float G = (cosTheta_q * cosTheta_x) / (distToLight * distToLight);
    Vector3 BRDF_x = surfaceIntegrator->evaluateBRDF(reservoir.wo, wi, reservoir.hitMat, reservoir.n);
    Vector3 Le = reservoir.lightMat->emission * reservoir.lightMat->colour;

    float lightPDF = (1.0f / reservoir.lightArea) * (1.0f / emissiveObjects.size());

    float candidateWeight = reservoir.targetPDF / reservoir.candidatePDF;
    float weightAdjustment = reservoir.weightSum / (reservoir.sampleCount * candidateWeight);

    return ((BRDF_x * Le * G) / lightPDF) * weightAdjustment;
}

void CPUPT::reservoirUpdate(Reservoir &r, Reservoir &candidate, float weight) const {
    r.weightSum += weight;
    r.sampleCount++;

    if (dist(rng) < (weight / r.weightSum)) {
        r.candidatePosition = candidate.candidatePosition;
        // extra things im storing for now
        r.candidatePDF = candidate.candidatePDF;
        r.candidateEmission = candidate.candidateEmission;
        r.candidateNormal = candidate.candidateNormal;
        r.lightMat = candidate.lightMat;
        r.lightArea = candidate.lightArea;
        r.empty = false;
    }
}

void CPUPT::restirDirectLighting(Ray &ray, Material *sampledMat, int x, int y) const {
    Reservoir reservoir = {};
    reservoirReSTIR[y * internalResX + x] = reservoir;
    if (emissiveObjects.size() == 0) return;

    std::uniform_int_distribution<int> lightDist(0, emissiveObjects.size() - 1);

    Vector3 rayPos = ray.getPos();
    Vector3 n = ray.getNormal();
    Vector3 wo = -ray.getDir();
    reservoir.rayPos = rayPos;
    reservoir.n = n;
    reservoir.wo = wo;
    reservoir.hitMat = sampledMat;

    // M = config.lightSamples
    for (int i = 0; i < config.candidateSamples; i++) {
        reservoir.sampleCount++;

        SceneObject *light = emissiveObjects[lightDist(rng)];
        Material *lightMat = light->getMaterial(); // need to adjust to take into account the triangles specific material
        Vector3 lightPoint = light->samplePoint(dist(rng), dist(rng));
        Vector3 wi = rayPos - lightPoint;
        float distToLight = wi.length();
        wi.normalise();
        float cosTheta_q = std::max(0.0f, n.dot(-wi));
        float cosTheta_x = std::max(0.0f, light->getNormal(lightPoint).dot(wi));
        if (cosTheta_q == 0 || cosTheta_x == 0) continue; // light point is facing away or behind hit point
        // = hitBRDF * emission * (distance^2 * cosTheta)
        // Pq(x) = hitPoint BRDF
        Vector3 BRDF_x = surfaceIntegrator->evaluateBRDF(wo, wi, sampledMat, n);
        // G(x) = cosTheta(x) * (cosTheta(q) / (x - q)^2
        float G = (cosTheta_x * cosTheta_q) / (distToLight * distToLight);
        // Le(x) = emission of light point
        Vector3 Le = lightMat->colour * lightMat->emission;

        // PDF of light point = (1 / lightArea) * (1 / numLights)
        float lightArea = light->getArea();
        float lightPDF = (1.0f / lightArea) * (1.0f / emissiveObjects.size());
        // (BRDF(q,x) * Le(x) * G(q,x)) / (1 / surfaceArea)
        Vector3 contribution = BRDF_x * Le * G; // unshadowed path contribution

        float targetPDF = Vector3::length(contribution);
        float candidateWeight = targetPDF / lightPDF; // w_i = p̂(x_i)/p(x_i)

        reservoir.weightSum += candidateWeight;
        if (dist(rng) < candidateWeight / reservoir.weightSum) {
            reservoir.candidatePosition = lightPoint;
            reservoir.distToLight = distToLight;
            // extra things im storing for now
            reservoir.targetPDF = targetPDF;
            reservoir.candidatePDF = lightPDF;
            reservoir.candidateEmission = light->getMaterial()->emission;
            reservoir.candidateNormal = light->getNormal(lightPoint);
            reservoir.lightMat = lightMat;
            reservoir.lightArea = lightArea;
            reservoir.empty = false;
        }
    }
    reservoirReSTIR[y * internalResX + x] = reservoir;
}

void CPUPT::restirSpatioTemporal(int xstart, int xend, int ystart, int yend, int its, int currentRay, std::mutex &mutex) const {
    // iterate over each pixel

    for (int y = ystart; y <= yend; y++) {
        for (int x = xstart; x <= xend; x++) {
            if (config.ReSTIR) {
                Reservoir currentReservoir = reservoirReSTIR[y * internalResX + x];

                if (currentReservoir.hitMat != nullptr) {
                    // Evaluate visibility for initial candidates
                    Vector3 wi = currentReservoir.candidatePosition - currentReservoir.rayPos;
                    float distToLight = Vector3::length(wi);
                    Vector3::normalise(wi);
                    Ray shadowRay(currentReservoir.rayPos, wi);
                    shadowRay.march(0.01f);
                    BVHNode::BVHResult shadowResult = rootNode->searchBVHTreeScene(shadowRay);
                    if (shadowResult.node == nullptr || !shadowResult.node->getLeaf() || !(
                            shadowResult.close < distToLight + 0.01f && shadowResult.close > distToLight - 0.01f)) {
                        // not visible
                        currentReservoir.weightSum = 0.0f;
                    }

                    // Temporal Resampling
                    // WIP

                    // Spatial Resampling
                    for (int i = 0; i < config.spatialSamplesK; i++) {
                        Pixel candidate = neighbourCandidate(x, y); // select random pixel in specified radius
                        Reservoir neighbourReservoir = reservoirReSTIR[candidate.y * internalResX + candidate.x];

                        if (neighbourReservoir.empty) {
                            continue;
                        }

                        // compute unshadowed contribtuion of neighbour sample at current point
                        Vector3 wi = currentReservoir.rayPos - neighbourReservoir.candidatePosition;
                        float distToLight = wi.length();
                        Vector3::normalise(wi);

                        Vector3 BRDF_x = surfaceIntegrator->evaluateBRDF(currentReservoir.wo, wi, currentReservoir.hitMat, currentReservoir.n);
                        Vector3 Le = neighbourReservoir.lightMat->emission * neighbourReservoir.lightMat->colour;

                        float cosTheta_q = std::max(0.0f, currentReservoir.n.dot(wi));
                        float cosTheta_x = std::max(0.0f, currentReservoir.candidateNormal.dot(-wi));
                        float G = (cosTheta_q * cosTheta_x) / (distToLight * distToLight);
                        if (cosTheta_q == 0 || cosTheta_x == 0) continue; // neighbour candidate point is facing away or behind hit point

                        Vector3 finalContribution = BRDF_x * Le * G;
                        float targetPDFNew = Vector3::length(finalContribution);

                        float neighborAvgWeight = neighbourReservoir.weightSum / neighbourReservoir.sampleCount;
                        float currentAvgWeight = currentReservoir.weightSum / currentReservoir.sampleCount;

                        float misWeight = (neighbourReservoir.weightSum * neighbourReservoir.sampleCount) / (targetPDFNew / neighbourReservoir.targetPDF);

                        if (config.unbiased) {
                            // unbiased - rejection based on shadow ray - slower

                            // Shadow test
                            Ray shadowRay(currentReservoir.rayPos + wi * 0.01f, wi);
                            BVHNode::BVHResult shadowResult = rootNode->searchBVHTreeScene(shadowRay);
                            if (shadowResult.node == nullptr || !shadowResult.node->getLeaf() ||
                                !(shadowResult.close < distToLight + 0.05f && shadowResult.close > distToLight - 0.05f)) {
                                // occulsion
                                misWeight = 0.0f;
                            }
                        } else {
                            // biased approach - rejection based on normal and depth - very fast
                            Vector3 currentNormal = normalBuffer[y * internalResX + x];
                            Vector3 neighbourNormal = normalBuffer[candidate.y * internalResX + candidate.x];
                            float dot = Vector3::dot(currentNormal, neighbourNormal);
                            if (dot < 0.75) misWeight = 0.0f;

                            float currentDepth = depthBuffer[y * internalResX + x];
                            float neighbourDepth = depthBuffer[candidate.y * internalResX + candidate.x];
                            float depthFactor = currentDepth / neighbourDepth;
                            if (depthFactor < 0.9 || depthFactor > 1.10) misWeight = 0.0f;
                        }

                        // Update the reservoir
                        reservoirUpdate(currentReservoir, neighbourReservoir, misWeight);
                    }

                    // Compute final contribution
                    if (!currentReservoir.empty && currentReservoir.weightSum != 0) {
                        // Shadow ray test
                        Vector3 BRDF_x = surfaceIntegrator->evaluateBRDF(currentReservoir.wo, wi, currentReservoir.hitMat, currentReservoir.n);
                        Vector3 Le = currentReservoir.lightMat->emission * currentReservoir.lightMat->colour;

                        float cosTheta_q = std::max(0.0f, currentReservoir.n.dot(wi));
                        float cosTheta_x = std::max(0.0f, currentReservoir.candidateNormal.dot(-wi));
                        float G = (cosTheta_q * cosTheta_x) / (distToLight * distToLight);

                        Vector3 finalContribution = BRDF_x * Le * G;
                        float targetPDF = Vector3::length(BRDF_x * Le * G);
                        float finalWeight = (currentReservoir.weightSum / currentReservoir.sampleCount) / targetPDF;

                        // Update un-normalised buffer
                        hdr[y * internalResX + x] += finalContribution * finalWeight;
                    }
                } // end empty check
            } // end ReSTIR
            // normalise progress buffer
            lum[y * internalResX + x] = hdr[y * internalResX + x] / (
                                            static_cast<float>(currentRay) + static_cast<float>(its) * static_cast<float>(config.raysPerPixel));
        } // end x
    } // end y
}

Pixel CPUPT::neighbourCandidate(int x, int y) const {
    int xRand = dist(rng);
    while (x + xRand < 0 || x + xRand >= internalResX || x + xRand == x) { xRand = spatialDist(rng); }
    int yRand = dist(rng);
    while (y + yRand < 0 || y + yRand >= internalResY || y + yRand == y) { yRand = spatialDist(rng); }

    return {x + xRand, y + yRand};
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
    debugPixelInfo(x, y);

    // Ray cast to find clicked object
    Ray ray;
    ray.reset();
    ray.setDebugTrue(); // for debug
    // Camera (primary) ray origin:

    ray.getPos().set(camera->getPos());
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
    int x = screenX / upScale;
    int y = screenY / upScale;
    std::cout << "X: " << x << ", Y: " << y << std::endl;

    // print reservoir debuf info
    Reservoir debugReservoir = reservoirReSTIR[y * internalResX + x];
    std::cout << "Normal in reservoir = ";
    debugReservoir.n.print();

    // Ray cast to find clicked object
    Ray ray;
    ray.reset();
    ray.setDebugTrue();
    // Camera (primary) ray origin:

    ray.getPos().set(camera->getPos());
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

    for (int currentBounce = 0; currentBounce <= 0; currentBounce++) {
        std::cout << "----------------------------" << std::endl;
        std::cout << "Bounce: " << currentBounce << std::endl;
        // Intersect scene using BVH
        BVHNode::BVHResult leafNode = rootNode->searchBVHTreeScene(ray);
        ray.setTriangle(leafNode.triangle);
        ray.getBCoords().set(leafNode.bCoords);
        if (leafNode.node == nullptr || !leafNode.node->getLeaf()) {
            // Ray intersects nothing, break
            break;
        }

        // Move the ray to the exact hit point
        if (ray.getInternal()) std::cout << "Ray Internal" << std::endl;
        else std::cout << "Ray External" << std::endl;

        std::cout << "Distance Close: " << leafNode.close << std::endl;
        std::cout << "Distance Far: " << leafNode.far << std::endl;

        SceneObject *hitObject = leafNode.node->getSceneObject();
        if (!ray.getInternal()) {
            // outside of an object - march to entry
            ray.march(leafNode.close);
        } else {
            // inside of an object - march to far side
            ray.march(leafNode.far);
        }

        hitObject->getNormal(ray); // sets ray.getNormal()
        ray.getPos().set(ray.getPos()); // Set new ray origin
        ray.setHitObject(hitObject);

        std::cout << "Surface normal: ";
        ray.getNormal().print();

        // Compute material properties at the hitpoint (mesh objects)
        Material *sampledMat = materialManager->sampleMatFromHitPoint(ray, hitObject->getMaterial(ray), hitObject);

        // -------------------------------------------------------------
        // ReSTIR Direct Illumination (first bounce only)
        // -------------------------------------------------------------

        if (currentBounce == 0 && config.ReSTIR) {
            delete reservoirReSTIR[y * internalResX + x].hitMat;
            Material *ReSTIRsampledMat = materialManager->sampleMatFromHitPoint(ray, sampledMat, hitObject);
            restirDirectLighting(ray, ReSTIRsampledMat, x, y);
        }

        // -------------------------------------------------------------
        // Multiple Importance Sampling
        // -------------------------------------------------------------
        // 1) Sample new direction: reflection or refraction and compute BRDF and PDF
        float randomSample = dist(rng);
        float p_specular = sampledMat->metallic;
        float p_transmission = sampledMat->transmission * (1.0f - sampledMat->metallic);
        float p_diffuse = 1.0f - (p_specular + p_transmission);
        // probabilites should add up to 1

        Vector3 wo = -ray.getDir(); // outgoing direction
        Vector3 wi;

        Vector3 n = ray.getNormal();
        Vector3 newThroughput;

        // specular caused by IOR
        float cosTheta = std::abs(ray.getDir().dot(ray.getNormal()));
        float R0 = surfaceIntegrator->fresnelSchlickIOR(cosTheta, sampledMat->IOR); // reflection portion
        float randomSample2 = dist(rng); // a second sample
        // ----------------------

        bool isMesh = hitObject->isMesh();
        if (ray.getInternal()) {
            // Refraction
            // Continue refraction from previous bounce
            wi = directionSampler->RefractionDirection(ray, *hitObject);
            newThroughput = surfaceIntegrator->computeThroughput(wo, wi, sampledMat, n, R0, refrecation, true);
        } else {
            if (randomSample <= p_specular) {
                // Specular (Metallic)
                wi = directionSampler->SpecularDirection(ray, *hitObject, false);
                newThroughput = surfaceIntegrator->computeThroughput(wo, wi, sampledMat, n, R0, metallic, false);
            } else if (randomSample <= p_specular + p_transmission) {
                // Blend in the possibility of refraction based on (transmission * (1 - metallic))
                if (randomSample2 < R0) {
                    // Specular (Glass)
                    wi = directionSampler->SpecularDirection(ray, *hitObject, false);
                    newThroughput = surfaceIntegrator->computeThroughput(wo, wi, sampledMat, n, R0, specularFresnel, false);
                } else {
                    // Refraction
                    wi = directionSampler->RefractionDirection(ray, *hitObject);
                    newThroughput = surfaceIntegrator->computeThroughput(wo, wi, sampledMat, n, R0, refrecation, false);
                }
            } else if (randomSample <= p_specular + p_transmission + p_diffuse) {
                if (randomSample2 < R0) {
                    // Specular Diffuse
                    wi = directionSampler->SpecularDirection(ray, *hitObject, false);
                    newThroughput = surfaceIntegrator->computeThroughput(wo, wi, sampledMat, n, R0, specularFresnel, false);
                } else {
                    // Dielectric reflection
                    wi = directionSampler->DiffuseDirection(ray, *hitObject, false); // sample direction
                    newThroughput = surfaceIntegrator->computeThroughput(wo, wi, sampledMat, n, R0, diffuse, false);
                }
            } else {
                std::cout << "Probabilities dont add up: " << randomSample << std::endl;
                break;
            }
        }

        // 2) Update throughput with BRDF and PDF
        throughput = throughput * newThroughput;

        // 3) Update colour
        finalColour = finalColour + throughput * (sampledMat->colour * sampledMat->emission);
        //finalColour.sanitize();
        delete sampledMat;

        // -------------------------------------------------------------
        // Russian roulete termination
        // -------------------------------------------------------------

        float RR = std::min(std::max(throughput.maxComponent(), 0.1f), 1.0f);

        // Ensure minimum number of bounces completed
        if (currentBounce > config.minBounces) {
            if (dist(rng) > RR) {
                // Add the remaining contribution before termination
                //finalColour = finalColour + throughput * (mat->colour * mat->emission);
                break; // Terminate the ray path early
            }
            // Scale the throughput to maintain an unbiased estimator
            throughput = throughput / RR;
        }

        // Prepare for next bounce
        ray.getDir().set(wi);
        ray.march(0.01);
    } // end for bounceDepth

    // -------------------------------------------------------------
    // Ray terminated. Accumulate into the buffer.
    // -------------------------------------------------------------
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