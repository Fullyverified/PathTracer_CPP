#include "Render.h"

#include <atomic>
#include <algorithm>
#include <chrono>
#include <future>
#include <iostream>
#include <thread>
#include <mutex>
#include <numbers>
#include <vector>

#include "Config.h"

#include "Ray.h"
#include "BVHNode.h"
//#include "SDLWindow.h"

Render::Render(Camera &cam) : cam(cam), resX(config.resX), resY(config.resX / (config.aspectX / config.aspectY)), internalResX(config.resX / config.upScale),
                              internalResY(resY / config.upScale), boundsX(0, 0), boundsY(0, 0), dist(0.0f, 1.0f), iterations(0), running(true),
                              sceneUpdated(false), camMoved(true), lockInput(false), numThreads(0) {
    int res = internalResX * internalResY;
    primaryRay.resize(res, nullptr);
    secondaryRay.resize(res, nullptr);
    lumR.resize(res, 0.0f);
    lumG.resize(res, 0.0f);
    lumB.resize(res, 0.0f);
    absR.resize(res, 0.0f);
    absG.resize(res, 0.0f);
    absB.resize(res, 0.0f);
    std::cout << "External: resX:" << resX << ", internal: resY: " << resY << std::endl;
    std::cout << "Internal: resX:" << internalResX << ", internal: resY: " << internalResY << std::endl;
}

void Render::renderLoop(std::vector<SceneObject *> &sceneobjectsList, Camera &cam, SDLWindow &window) {
    // initialise objects
    std::mutex mutex;
    std::vector<std::future<void> > threads;
    // create all ray and luminance vector objects
    std::cout << "Constructing Objects: " << std::endl;
    intialiseObjects();
    std::cout << "Avaliable Threads: " << std::thread::hardware_concurrency() << std::endl;

    std::cout << "Constructing BVH: " << std::endl;
    constructBVHST(sceneobjectsList);
    BVHNode *BVHrootNode = BVHNodes.at(0);

    // render loop code
    while (running) {
        numThreads = config.threads > 0 ? config.threads : std::thread::hardware_concurrency();
        int segments = std::round(std::sqrt(numThreads));

        if (camMoved) {
            if (sceneUpdated) {
                std::cout << "Reconstructing BVH: " << std::endl;
                constructBVHST(sceneobjectsList);
                BVHNode *BVHrootNode = BVHNodes.at(0);
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

            aspectRatio = static_cast<float>(internalResX) / internalResY;
            // convert fOV to radians
            fovYRad = (config.fOV * std::numbers::pi) / 180.0f;
            fovXRad = 2 * atan(tan(fovYRad / 2) * aspectRatio);
            // compute scaling Factors
            scaleX = tan(fovXRad / 2);
            scaleY = tan(fovYRad / 2);

            // primary rays
            auto startTimePR = std::chrono::high_resolution_clock::now();
            for (int j = 0; j < segments; j++) {
                for (int i = 0; i < segments; i++) {
                    boundsX = threadSegments(internalResX, segments, boundsX, i);
                    boundsY = threadSegments(internalResY, segments, boundsY, j);
                    threads.emplace_back(std::async(std::launch::async, &Render::computePrimaryRay, this, cam, boundsX.first, boundsX.second, boundsY.first,
                                                    boundsY.second,
                                                    std::ref(*BVHrootNode), std::ref(mutex)));
                }
            }
            for (std::future<void> &thread: threads) {
                thread.get(); // Blocks until the thread completes its task
            }
            threads.clear();
            auto durationTimeMT = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimePR);
            std::cout << "PrimaryRay Time: " << durationTimeMT.count() << "ms" << std::endl;
            iterations = 1;
            camMoved = false;
            sceneUpdated = false;
        }
        //---------------------

        // secondary rays
        auto startTimeSR = std::chrono::high_resolution_clock::now();
        std::cout << "Starting Secondary Rays: " << std::endl;
        for (int j = 0; j < segments; j++) {
            for (int i = 0; i < segments; i++) {
                boundsX = threadSegments(internalResX, segments, boundsX, i);
                boundsY = threadSegments(internalResY, segments, boundsY, j);
                int its = iterations;
                threads.emplace_back(std::async(std::launch::async, &Render::computeSecondaryRay, this,
                                                boundsX.first, boundsX.second, boundsY.first,
                                                boundsY.second,
                                                std::ref(*BVHrootNode), its, std::ref(mutex)));
            }
        }
        for (std::future<void> &thread: threads) {
            thread.get(); // Blocks until the thread completes its task
        }
        threads.clear();
        auto durationTimeSR = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeSR);
        std::cout << config.raysPerPixel * iterations << " Secondary Ray(s) Time: " << durationTimeSR.count() << "ms" << std::endl;
        iterations++;
        //-----------------------

        // tone mapping
        auto startTimeTM = std::chrono::high_resolution_clock::now();
        maxR = 0;
        maxG = 0;
        maxB = 0;
        for (int i = 0; i < internalResX * internalResY; i++) {
            // determine brightest amplitude in scene
            maxR = lumR[i] > maxR ? lumR[i] : maxR;
            maxG = lumG[i] > maxG ? lumG[i] : maxG;
            maxB = lumB[i] > maxB ? lumB[i] : maxB;
        }
        maxLuminance = (0.2126f * maxR + 0.7152f * maxG + 0.0722f * maxB) * config.ISO;
        for (int j = 0; j < segments; j++) {
            for (int i = 0; i < segments; i++) {
                boundsX = threadSegments(internalResX, segments, boundsX, i);
                boundsY = threadSegments(internalResY, segments, boundsY, j);
                threads.emplace_back(std::async(std::launch::async, &Render::toneMap, this, std::ref(maxLuminance),
                                                boundsX.first, boundsX.second, boundsY.first,
                                                boundsY.second, std::ref(mutex)));
            }
        }
        for (std::future<void> &thread: threads) {
            thread.get(); // Blocks until the thread completes its task
        }
        threads.clear();
        window.presentScreen(RGBBuffer, resX);
        auto durationTimeTM = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeTM);
        std::cout << "Tone Mapping And Present Time: " << durationTimeTM.count() << "ms" << std::endl;
        //-----------------------
    }
}

void Render::computePixels(std::vector<SceneObject *> &sceneobjectsList, Camera &cam) {
    // Create Window
    SDLWindow window;
    int width = resX;
    int height = resY;
    window.createWindow(width, height);
    window.createRenderer();
    window.initializeTexture(width, height);

    // render loop
    std::thread renderThread(&Render::renderLoop, this, std::ref(sceneobjectsList), std::ref(cam), std::ref(window));

    // Main event loop
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            }
            if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_DELETE) {
                    lockInput = lockInput == false ? true : false;
                    window.setRelativeMouse();
                    std::cout << "locking input" << std::endl;
                }
            }
        }
        const Uint8 *inputState = SDL_GetKeyboardState(NULL);
        SDL_GetRelativeMouseState(&mouseX, &mouseY);

        if (inputState[SDL_SCANCODE_UP]) {
            config.increaeISO();
        }
        if (inputState[SDL_SCANCODE_DOWN]) {
            config.decreaseISO();
        }
        if (inputState[SDL_SCANCODE_RIGHT]) {
            config.resetISO();
        }
        if (inputState[SDL_SCANCODE_ESCAPE]) {
            running = false;
        }

        if (!lockInput) {
            if (inputState[SDL_SCANCODE_KP_PLUS]) {
                config.increaeFOV();
                cam.reInitilize();
                camMoved = true;
            }
            if (inputState[SDL_SCANCODE_KP_MINUS]) {
                config.decreaeFOV();
                cam.reInitilize();
                camMoved = true;
            }
            if (inputState[SDL_SCANCODE_W]) {
                camMoved = true;
                cam.moveForward(0.1f);
            }
            if (inputState[SDL_SCANCODE_S]) {
                camMoved = true;
                cam.moveBackward(0.1f);
            }
            if (inputState[SDL_SCANCODE_A]) {
                camMoved = true;
                cam.moveLeft(0.1f);
            }
            if (inputState[SDL_SCANCODE_D]) {
                camMoved = true;
                cam.moveRight(0.1f);
            }
            if (inputState[SDL_SCANCODE_E]) {
                camMoved = true;
                cam.moveUp(0.1f);
            }
            if (inputState[SDL_SCANCODE_Q]) {
                camMoved = true;
                cam.moveDown(0.1f);
            }

            if (mouseX != 0 || mouseY != 0) {
                cam.updateDirection(mouseX, mouseY);
                camMoved = true;
            }
        }

        // Optional: Limit frame rate or add delay if necessary
        SDL_Delay(1); // ~60 FPS (1000 ms / 16 ms = 60.25 FPS)
    }

    renderThread.join();

    std::cout << "Deleting Objects" << std::endl;
    deleteObjects(); // delete all ray and luminance vectors
    window.destroyWindow(); // delete SDL objects
}

void Render::toneMap(float &maxLuminance, int xstart, int xend, int ystart, int yend, std::mutex &mutex) {
    for (int x = xstart; x <= xend; x++) {
        for (int y = ystart; y <= yend; y++) {
            float red = lumR[y * internalResX + x];
            float green = lumG[y * internalResX + x];
            float blue = lumB[y * internalResX + x];

            float luminance = 0.2126f * red + 0.7152f * green + 0.0722f * blue;

            // Extended Reinhard Tone Mapping - returns value [0, 1]
            float mappedLuminance = (luminance * (1 + (luminance / (maxLuminance * maxLuminance)))) / (1 + luminance);

            if (luminance > 0) {
                red *= mappedLuminance / luminance;
                green *= mappedLuminance / luminance;
                blue *= mappedLuminance / luminance;
            }

            red *= config.ISO;
            green *= config.ISO;
            blue *= config.ISO;

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

            // Upscale and store in RGB buffer considering aspect ratio
            for (int i = 0; i < config.upScale; i++) {
                for (int j = 0; j < config.upScale; j++) {
                    int outX = static_cast<int>(x * config.upScale + i);
                    int outY = static_cast<int>(y * config.upScale + j);
                    int offset = (outY * resX + outX) * 3;

                    RGBBuffer[offset] = red;
                    RGBBuffer[offset + 1] = green;
                    RGBBuffer[offset + 2] = blue;
                }
            }
        }
    }
}

void Render::computePrimaryRay(Camera cam, int xstart, int xend, int ystart, int yend, BVHNode &rootNode, std::mutex &mutex) const {
    for (int y = ystart; y <= yend; y++) {
        for (int x = xstart; x <= xend; x++) {
            Ray *ray = primaryRay[internalResX * y + x];
            ray->getOrigin().set(cam.getPos());
            ray->getPos().set(cam.getPos());
            // calculate position of pixel on image plane
            Vector3 pixelPosPlane(((((x + 0.5f) / internalResX) * 2) - 1) * aspectRatio, 1 - (((y + 0.5f) / internalResY) * 2), 0);
            Vector3 pixelPosScene(pixelPosPlane.getX() * cam.getPlaneWidth() / 2, pixelPosPlane.getY() * cam.getPlaneHeight() / 2, 0);
            // point ray according to pixel position (ray starts from camera origin)
            ray->getDir().set(
                cam.getDir().getX() + cam.getRight().getX() * pixelPosScene.getX() + cam.getUp().getX() * pixelPosScene.getY(),
                cam.getDir().getY() + cam.getRight().getY() * pixelPosScene.getX() + cam.getUp().getY() * pixelPosScene.getY(),
                cam.getDir().getZ() + cam.getRight().getZ() * pixelPosScene.getX() + cam.getUp().getZ() * pixelPosScene.getY());
            ray->getDir().normalise();
            BVHNode *leafNode = rootNode.searchBVHTree(*ray);
            ray->setHit(false);
            if (leafNode != nullptr) {
                // did we make it to a leaf node?
                SceneObject *BVHSceneObject = leafNode->getSceneObject();
                std::vector<float> objectDistance = BVHSceneObject->getIntersectionDistance(*ray);
                float distanceClose = objectDistance[0];
                ray->march(distanceClose); // march the ray to the object
                ray->getHitPoint().set(ray->getPos());
                ray->setHit(true);
                ray->setHitObject(BVHSceneObject);
            }
        }
    }
}

void Render::computeSecondaryRay(int xstart, int xend, int ystart, int yend, BVHNode &rootNode, int its, std::mutex &mutex) const {
    std::vector<std::vector<float> > depthRed(config.bounceDepth + 1, std::vector<float>(4, 0.0f));
    std::vector<std::vector<float> > depthGreen(config.bounceDepth + 1, std::vector<float>(4, 0.0f));
    std::vector<std::vector<float> > depthBlue(config.bounceDepth + 1, std::vector<float>(4, 0.0f));
    std::vector<float> lum(3, 0.0f);
    std::vector<float> col(3, 0.0f);
    for (int currentRay = 1; currentRay <= config.raysPerPixel; currentRay++) {
        for (int y = ystart; y <= yend; y++) {
            for (int x = xstart; x <= xend; x++) {
                Ray *primRay = primaryRay[internalResX * y + x];
                if (primRay->getHit()) {
                    // store primary ray depth information
                    for (int i = 0; i < config.bounceDepth + 1; i++) {
                        // reset hit bool
                        for (int j = 0; j <= 3; j++) {
                            depthRed[i][j] = 0;
                            depthGreen[i][j] = 0;
                            depthBlue[i][j] = 0;
                        }
                    }
                    primRay->getHitObject()->getNormal(*primRay); // update normal vector
                    float lambertCosineLaw = std::abs(primRay->getNormal().dot(primRay->getDir())); // dot product of object normal and ray direction
                    lum = primRay->getHitObject()->getLum();
                    col = primRay->getHitObject()->getCol();
                    depthRed[0][0] = lum[0]; // object brightness
                    depthRed[0][1] = lambertCosineLaw;
                    depthRed[0][2] = col[0];
                    depthRed[0][3] = 1; // boolean hit
                    depthGreen[0][0] = lum[1];
                    depthGreen[0][1] = lambertCosineLaw;
                    depthGreen[0][2] = col[1];
                    depthGreen[0][3] = 1;
                    depthBlue[0][0] = lum[2];
                    depthBlue[0][1] = lambertCosineLaw;
                    depthBlue[0][2] = col[2];
                    depthBlue[0][3] = 1;
                    // initialize secondary ray
                    Ray *nthRay = secondaryRay[internalResX * y + x];
                    nthRay->initialize(*primRay);
                    for (int currentBounce = 1; currentBounce <= config.bounceDepth; currentBounce++) {
                        // BOUNCES PER RAY
                        float randomSample = dist(rng); // monte carlo sampling
                        if (randomSample >= nthRay->getHitObject()->getTransp()) {
                            sampleReflectionDirection(*nthRay, *nthRay->getHitObject(), false);
                        } else {
                            sampleRefractionDirection(*nthRay, *nthRay->getHitObject(), false);
                        }
                        BVHNode *leafNode = rootNode.searchBVHTree(*nthRay);
                        //nthRay->setHit(false);
                        if (leafNode != nullptr && leafNode->getSceneObject() != nullptr) {
                            SceneObject *BVHSceneObject = leafNode->getSceneObject();
                            std::vector<float> objectDistance = BVHSceneObject->getIntersectionDistance(*nthRay);
                            float distanceClose = objectDistance[0];
                            nthRay->march(distanceClose);
                            nthRay->getHitPoint().set(nthRay->getPos());
                            //nthRay->setHit(true);
                            nthRay->setHitObject(BVHSceneObject);
                            nthRay->getOrigin().set(nthRay->getPos());
                            // store hit data
                            BVHSceneObject->getNormal(*nthRay); // update normal vector
                            lambertCosineLaw = std::abs(nthRay->getNormal().dot(nthRay->getDir()));
                            lum = BVHSceneObject->getLum();
                            col = BVHSceneObject->getCol();
                            depthRed[currentBounce][0] = lum[0];
                            depthRed[currentBounce][1] = lambertCosineLaw;
                            depthRed[currentBounce][2] = col[0];
                            depthRed[currentBounce][3] = 1;
                            depthGreen[currentBounce][0] = lum[1];
                            depthGreen[currentBounce][1] = lambertCosineLaw;
                            depthGreen[currentBounce][2] = col[1];
                            depthGreen[currentBounce][3] = 1;
                            depthBlue[currentBounce][0] = lum[2];
                            depthBlue[currentBounce][1] = lambertCosineLaw;
                            depthBlue[currentBounce][2] = col[2];
                            depthBlue[currentBounce][3] = 1;
                        }
                    }
                    // sum up ray depth information
                    float redAmplitude = 0;
                    float greenAmplitude = 0;
                    float blueAmplitude = 0;
                    for (int index = depthRed.size() - 1; index >= 0; index--) {
                        if (depthRed[index][3] == 1) {
                            redAmplitude = ((depthRed[index][0] + redAmplitude) * depthRed[index][1]) * depthRed[index][2];
                        }
                        if (depthGreen[index][3] == 1) {
                            greenAmplitude = ((depthGreen[index][0] + greenAmplitude) * depthGreen[index][1]) * depthGreen[index][2];
                        }
                        if (depthBlue[index][3] == 1) {
                            blueAmplitude = ((depthBlue[index][0] + blueAmplitude) * depthBlue[index][1]) * depthBlue[index][2];
                        }
                    }
                    absR[y * internalResX + x] += redAmplitude;
                    absG[y * internalResX + x] += greenAmplitude;
                    absB[y * internalResX + x] += blueAmplitude;
                    lumR[y * internalResX + x] = absR[y * internalResX + x] / (static_cast<float>(currentRay) * its);
                    lumG[y * internalResX + x] = absG[y * internalResX + x] / (static_cast<float>(currentRay) * its);
                    lumB[y * internalResX + x] = absB[y * internalResX + x] / (static_cast<float>(currentRay) * its);
                }
            }
        }
    }
}

thread_local std::mt19937 Render::rng(std::random_device{}());

void Render::sampleRefractionDirection(Ray &ray, SceneObject &sceneObject, bool flipNormal) const {
    sceneObject.getNormal(ray);
    // refraction
    float n1 = 1.0003f; // refractive index of air
    float n2 = sceneObject.getRefrac();
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
        Vector3 refraction(
            ray.getDir().getX() * (n1 / n2) + ((n1 / n2) * cosTheta1 - cosTheta2) * ray.getNormal().getX(),
            ray.getDir().getY() * (n1 / n2) + ((n1 / n2) * cosTheta1 - cosTheta2) * ray.getNormal().getY(),
            ray.getDir().getZ() * (n1 / n2) + ((n1 / n2) * cosTheta1 - cosTheta2) * ray.getNormal().getZ());
        ray.getDir().set(refraction);
        ray.getDir().normalise();
        ray.updateOrigin(sceneObject.getIntersectionDistance(ray)[1]); // march the ray to the other side of the object
        n1 = sceneObject.getRefrac();
        n2 = 1.0003;
        // cosine of incident angle
        sceneObject.getNormal(ray); // update normal
        cosTheta1 = -(ray.getNormal().dot(ray.getDir()));
        sinTheta1 = std::sqrt(1.0f - cosTheta1 * cosTheta1);
        sinTheta2 = (n1 / n2) * sinTheta1;
        while (sinTheta2 >= 1) {
            sampleReflectionDirection(ray, sceneObject, true); // inside object so flipped normal
            ray.updateOrigin(-0.01f); // undo the march from the previous method
            ray.updateOrigin(sceneObject.getIntersectionDistance(ray)[1]); // march the ray to the other side of the object
            // recalculate the sin of the angle to work out if the ray still has total internal reflection or not
            sceneObject.getNormal(ray); // update normal
            cosTheta1 = -(ray.getNormal().dot(ray.getDir()));
            sinTheta1 = std::sqrt(1.0f - cosTheta1 * cosTheta1);
            sinTheta2 = (n1 / n2) * sinTheta1;
        }
        cosTheta2 = std::sqrt(1.0f - sinTheta2 * sinTheta2);
        sceneObject.getNormal(ray); // update normal
        refraction.set(
            ray.getDir().getX() * (n1 / n2) + ((n1 / n2) * cosTheta1 - cosTheta2) * ray.getNormal().getX(),
            ray.getDir().getY() * (n1 / n2) + ((n1 / n2) * cosTheta1 - cosTheta2) * ray.getNormal().getY(),
            ray.getDir().getZ() * (n1 / n2) + ((n1 / n2) * cosTheta1 - cosTheta2) * ray.getNormal().getZ());
        ray.getDir().normalise();
        ray.updateOrigin(sceneObject.getIntersectionDistance(ray)[1] + 0.01f);
    }
    //std::cout<<"Relfection Direction Time: "<<refractionDirectionTime.count()<<"ns"<<std::endl;
}

void Render::sampleReflectionDirection(Ray &ray, SceneObject &sceneObject, bool flipNormal) const {
    sceneObject.getNormal(ray); // update normal
    if (flipNormal) {
        ray.getNormal().flip();
    }
    // reflection direction
    float dotProduct = ray.getDir().dot(ray.getNormal());
    Vector3 reflection(
        ray.getDir().getX() - 2 * dotProduct * ray.getNormal().getX(),
        ray.getDir().getY() - 2 * dotProduct * ray.getNormal().getY(),
        ray.getDir().getZ() - 2 * dotProduct * ray.getNormal().getZ());

    // generate random direction
    // two randoms between 0 and 1
    float alpha = dist(rng);
    float gamma = dist(rng);
    // convert to sphereical coodinates
    alpha = std::acos(std::sqrt(alpha)); // polar angle - sqrt more likely to be near the pole (z axis)
    gamma = 2 * std::numbers::pi * gamma; // azimuthal angle

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
    Vector3 sampledD(
        random.getX() * tangentT.getX() + random.getY() * bitangent.getX() + random.getZ() * ray.getNormal().getX(),
        random.getX() * tangentT.getY() + random.getY() * bitangent.getY() + random.getZ() * ray.getNormal().getY(),
        random.getX() * tangentT.getZ() + random.getY() * bitangent.getZ() + random.getZ() * ray.getNormal().getZ());

    // bias the reflection direction with the random direction
    // biasedDirection = (1 - roughness) * reflectionDirection + roughness * randomDirection
    float roughness = sceneObject.getRough();
    ray.getDir().set(
        ((1 - roughness) * reflection.getX()) + roughness * sampledD.getX(),
        ((1 - roughness) * reflection.getY()) + roughness * sampledD.getY(),
        ((1 - roughness) * reflection.getZ()) + roughness * sampledD.getZ());
    ray.getDir().normalise();
    if (ray.getDir().dot(ray.getNormal()) < 0) {
        ray.getDir().flip();
    }
    ray.updateOrigin(0.01f); // march the ray a tiny amount to move it off the object
}

void Render::constructBVHST(const std::vector<SceneObject *> &sceneObjectsList) {
    // create leaf nodes
    for (SceneObject *sceneObject: sceneObjectsList) {
        std::pair<Vector3, Vector3> bounds = sceneObject->getBounds();
        BoundingBox *boundingBox = new BoundingBox(bounds.first, bounds.second);
        BVHNodes.emplace_back(new BVHNode(boundingBox, *sceneObject));
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
                cost = (combinedBox.getArea() / (BVHNodes.at(i)->getArea() + BVHNodes.at(j)->getArea())) * (BVHNodes.at(i)->getNumChildren() + BVHNodes.at(j)->getNumChildren());

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

void Render::constructBVHMT(const std::vector<SceneObject *> &sceneObjectsList) {
    // create leaf nodes
    for (SceneObject *sceneObject: sceneObjectsList) {
        std::pair<Vector3, Vector3> bounds = sceneObject->getBounds();
        BoundingBox *boundingBox = new BoundingBox(bounds.first, bounds.second);
        BVHNodes.emplace_back(new BVHNode(boundingBox, *sceneObject));
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
                threads.emplace_back(std::async(std::launch::async, &Render::findBestPair, this,
                                                std::ref(BVHNodes), start, end, std::ref(bestCost), std::ref(indexLeft), std::ref(indexRight),
                                                std::ref(bestLeft), std::ref(bestRight), std::ref(mutex)));
            }
        } else {
            for (int i = 0; i < BVHNodes.size(); i++) {
                int end = i + 1;
                threads.emplace_back(std::async(std::launch::async, &Render::findBestPair, this,
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

void Render::findBestPair(const std::vector<BVHNode *> &nodes, int start, int end, std::atomic<float> &globalBestCost, int &leftIndex, int &rightIndex,
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

void Render::BVHProfiling() {
    Ray ray1(Vector3(4.5, -2.4, -1), Vector3(0.1, 0, 1));
    ray1.getDir().normalise();
    bool hit = false;
    std::cout << "Searching BVH" << std::endl;
    auto startTime = std::chrono::high_resolution_clock::now();
    BVHNode *leafNode = BVHNodes.at(0)->searchBVHTree(ray1);
    if (leafNode != nullptr) {
        std::cout << "Intersection Test: " << std::endl;;
        leafNode->getSceneObject()->printType();
        std::cout << "SceneObject Pos";
        leafNode->getSceneObject()->getPos().print();
        hit = leafNode->getSceneObject()->objectCulling(ray1);
    } else { hit = false; }
    auto durationTime = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - startTime);
    std::cout << "Finished tree traversal: " << durationTime.count() << "ns" << std::endl;
    std::cout << "Hit: " << hit << std::endl;

    /*Ray ray2(Vector3(0.1, 0.1, 0.1), Vector3(1, 0, 0.1));
    ray2.getDir().normalise();
    hit = false;
    std::cout << "Searching BVH" << std::endl;
    startTime = std::chrono::high_resolution_clock::now();
    leafNode = BVHNodes.at(0)->searchBVHTree(ray2);
    if (leafNode != nullptr) {
        leafNode->getSceneObject()->printType();
        hit = leafNode->getSceneObject()->objectCulling(ray2);
    } else { hit = false; }
    durationTime = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - startTime);
    std::cout << "Finished tree traversal: " << durationTime.count() << "ns" << std::endl;
    std::cout << "Hit: " << hit << std::endl;*/
}

void Render::intialiseObjects() {
    for (int i = 0; i < internalResX * internalResY; i++) {
        secondaryRay[i] = new Ray();
        primaryRay[i] = new Ray();
    }
    RGBBuffer = new uint8_t[resX * resY * 3];
}

void Render::deleteObjects() {
    for (auto node: BVHNodes) {
        delete node;
    }
    BVHNodes.clear();

    for (int i = 0; i < resX * resY; i++) {
        delete primaryRay[i];
        delete secondaryRay[i];
    }
    delete RGBBuffer;
}

std::pair<int, int> Render::threadSegments(float resInput, int &numThreads, std::pair<int, int>, int step) {
    int res = resInput;
    int pixelsPerThread = res / numThreads; // number of pixels per thread
    int remainder = res % numThreads; // remaining pixels after division

    int start = step * pixelsPerThread + std::min(step, remainder);
    int end = (step + 1) * pixelsPerThread + std::min(step + 1, remainder) - 1;

    return std::make_pair(start, end);
}
