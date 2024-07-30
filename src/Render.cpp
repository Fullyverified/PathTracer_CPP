#include "Render.h"

#include <atomic>
#include <algorithm>
#include <chrono>
#include <future>
#include <iostream>
#include <thread>
#include <mutex>
#include <vector>

#include <Config.h>

Render::Render(Camera &cam) : primaryRayStep(config.primaryRayStep), secondaryRayStep(config.secondaryRayStep),
cam(cam), resX(cam.getResX()), resY(cam.getResY()), boundsX(0, 0), boundsY(0, 0), dist(0.0f, 1.0f) {
    primaryRay.resize(resX, std::vector<Ray*>(resY, nullptr));
    secondaryRay.resize(resX, std::vector<Ray*>(resY, nullptr));
    lumR.resize(resX, std::vector<float>(resY, 0.0f));
    lumG.resize(resX, std::vector<float>(resY, 0.0f));
    lumB.resize(resX, std::vector<float>(resY, 0.0f));
    absR.resize(resX, std::vector<float>(resY, 0.0f));
    absG.resize(resX, std::vector<float>(resY, 0.0f));
    absB.resize(resX, std::vector<float>(resY, 0.0f));
}

void Render::computePixels(std::vector<SceneObject *> &sceneobjectsList, Camera &cam) {

    int numThreads = std::thread::hardware_concurrency();
    numThreads = 1;
    std::mutex mutex;
    std::vector<std::future<void> > threads;
    std::cout << "Avaliable Threads: " << numThreads << std::endl;
    std::cout << "Constructing Objects: " << std::endl;
    // create all ray and luminance vector objects
    intialiseObjects();

    // construct BVH
    std::cout << "Constructing BVH: " << std::endl;
    constructBVHST(sceneobjectsList);
    BVHNode *BVHrootNode = BVHNodes.at(0);

    // primary rays
    auto startTimeMT = std::chrono::high_resolution_clock::now();

    int segments = std::round(std::sqrt(numThreads));
    for (int j = 0; j < segments; j++) {
        for (int i = 0; i < segments; i++) {
            boundsX = secondarySegments(cam.getResX(), segments, boundsX, i);
            boundsY = secondarySegments(cam.getResY(), segments, boundsY, j);
            threads.emplace_back(std::async(std::launch::async, &Render::computePrimaryRay, this,
                                            std::ref(cam), std::ref(primaryRay), boundsX.first, boundsX.second, boundsY.first, boundsY.second,
                                            std::ref(*BVHrootNode), std::ref(mutex)));
        }
    }
    for (std::future<void> &thread: threads) {
        thread.get(); // Blocks until the thread completes its task
    }
    threads.clear();
    auto durationTimeMT = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeMT);
    std::cout << "PrimaryRay Time: " << durationTimeMT.count() << "ms" << std::endl;

    /*// secondary rays
    startTimeMT = std::chrono::high_resolution_clock::now();

    for (int j = 0; j < segments; j++) {
        for (int i = 0; i < segments; i++) {
            boundsX = secondarySegments(cam.getResX(), segments, boundsX, i);
            boundsY = secondarySegments(cam.getResY(), segments, boundsY, j);
            threads.emplace_back(std::async(std::launch::async, &Render::computeSecondaryRay, this,
                                            std::ref(cam), std::ref(primaryRay), std::ref(secondaryRay), boundsX.first, boundsX.second, boundsY.first, boundsY.second,
                                            std::ref(*BVHrootNode), std::ref(mutex)));
        }
    }
    for (std::future<void> &thread: threads) {
        thread.get(); // Blocks until the thread completes its task
    }
    threads.clear();
    durationTimeMT = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTimeMT);
    std::cout<<config.raysPerPixel<<" Secondary Ray Time: "<<durationTimeMT.count()<<"ms"<<std::endl;*/

    printScreen();

    std::cout<<"Deleting Objects"<<std::endl;
    deleteObjects(); // delete all ray and luminance vectors
}

void Render::printScreen() {
    for (int x = 0; x < cam.getResX(); x++) {
        for (int y = 0; y < cam.getResY(); y++) {
            float lum = lumR[x][y] + lumG[x][y] + lumB[x][y];
            if (lum > 40) {
                std::cout<<"@@";
            }
            else if (lum > 15) {
                std::cout<<"##";
            }
            else if (lum > 2) {
                std::cout<<"xx";
            }
            else if (lum > 1.5) {
                std::cout<<"~~";
            }
            else if (lum > 1) {
                std::cout<<";;";
            }
            else if (lum > 0.5) {
                std::cout<<",,";
            }
            else if (lum <= 0.5) {
                std::cout<<"  ";
            }
        }
        std::cout<<"|"<<std::endl;
    }
}

void Render::computePrimaryRay(Camera &cam, std::vector<std::vector<Ray*>> &primaryRay, int xstart, int xend,int ystart, int yend, BVHNode &rootNode, std::mutex &mutex) const {
    for (int y = ystart; y <= yend; y++) {
        for (int x = xstart; x <= xend; x++) {
            std::cout<<"_____________"<<std::endl;
            std::cout<<"X: "<<x<<", Y: "<<y<<std::endl;
            Ray* ray = primaryRay[x][y];
            ray->getOrigin().set(cam.getPos());
            // calculate position of pixel on image plane
            Vector3 pixelPosPlane((((x + 0.5f) / cam.getResX()) * 2) - 1, 1 - (((y + 0.5f) / cam.getResX()) * 2), 0);
            Vector3 pixelPosScene(pixelPosPlane.getX() * cam.getPlaneWidth() / 2, pixelPosPlane.getY() * cam.getPlaneHeight() / 2, 0);
            // point ray to pixel position (ray starts from camera origin)
            ray->getDir().set(
                cam.getDir().getX() + cam.getRight().getX() * pixelPosScene.getX() + cam.getUp().getX() * pixelPosScene.getY(),
                cam.getDir().getY() + cam.getRight().getY() * pixelPosScene.getX() + cam.getUp().getY() * pixelPosScene.getY(),
                cam.getDir().getZ() + cam.getRight().getZ() * pixelPosScene.getX() + cam.getUp().getZ() * pixelPosScene.getY());
            ray->getDir().normalise();
            ray->getDir().print();
            ray->getPos().print();
            ray->march(0);
            BVHNode *leafNode = rootNode.searchBVHTree(*ray);
            if (leafNode != nullptr) {
                //std::cout<<"PR Leaf Node Found"<<std::endl;
                // did we make it to a leaf node?
                SceneObject *BVHSceneObject = leafNode->getSceneObject();
                BVHSceneObject->printType();
                std::vector<float> objectDistance = leafNode->getIntersectionDistance(*ray);
                float distanceClose = objectDistance[0];
                float distanceFar = objectDistance[1];
                BVHSceneObject->getBounds().first.print();
                BVHSceneObject->getBounds().second.print();
                std::cout<<"BoundingBox Bounds: "<<std::endl;
                leafNode->getBoundingBox()->getBounds().first.print();
                leafNode->getBoundingBox()->getBounds().second.print();
                std::cout<<"BVHdistanceClose: "<<distanceClose<<", BVHdistanceFar: "<<distanceFar<<std::endl;
                float distance = distanceClose;
                // DEBUGGING DISTANCE CLOSE AND DISTANCE FAR - wrong values?? ray never getting an intersection?? ray direction is correct.
                ray->march(distance - 0.05f); // march the ray to the objects bounding volume
                while (distance <= distanceFar && !primaryRay[x][y]->getHit()) {
                    ray->march(distance);
                    if (BVHSceneObject->intersectionCheck(*ray)) {
                        //std::cout<<"Primary Ray Hit"<<std::endl;
                        ray->getHitPoint().set(ray->getPos());
                        ray->setHit(true);
                        std::cout<<"Hit"<<std::endl;
                        ray->setHitObject(BVHSceneObject);
                        //std::lock_guard<std::mutex> lock(mutex);
                        lumR[x][y] = 10;
                        lumG[x][y] = 10;
                        lumB[x][y] = 10;
                    }
                    distance += config.primaryRayStep;
                }
            }
        }
    }
}

void Render::computeSecondaryRay(Camera &cam, std::vector<std::vector<Ray*> > &primaryRayV, std::vector<std::vector<Ray *> > &secondaryRayV, int xstart, int xend, int ystart, int yend, BVHNode &rootNode, std::mutex &mutex) const {
    /*std::cout<<"xtsart"<<xstart<<", xend"<<xend<<std::endl;
    std::cout<<"ytsart"<<ystart<<", yend"<<yend<<std::endl;
    std::cout<<"Initializing float vectors"<<std::endl;*/
    std::vector<std::vector<float>> depthRed(config.bounceDepth + 1, std::vector<float>(4, 0.0f));
    std::vector<std::vector<float>> depthGreen(config.bounceDepth + 1, std::vector<float>(4, 0.0f));
    std::vector<std::vector<float>> depthBlue(config.bounceDepth + 1, std::vector<float>(4, 0.0f));
    std::vector<float> lum(3, 0.0f);
    std::vector<float> col(3, 0.0f);
    for (int currentRay = 1; currentRay <= config.raysPerPixel; currentRay++) {
        for (int y = ystart; y <= yend; y++) {
            for (int x = xstart; x <= xend; x++) {
                /*std::cout<<"X: "<<x<<", Y: "<<y<<std::endl;
                std::cout<<"Making primary Ray"<<std::endl;*/
                Ray *primaryRay = primaryRayV[x][y];
                if (primaryRay->getHit()) {
                    //std::cout<<"Making secondary Ray"<<std::endl;
                    Ray *nthRay = secondaryRayV[x][y];
                    //std::cout<<"Reset bounce depth"<<std::endl;
                    for (int i = 0; i < config.bounceDepth + 1; i++) { // reset hit bool
                      depthRed[i][3] = 0;
                      depthGreen[i][3] = 0;
                      depthBlue[i][3] = 0;
                    }
                    nthRay->initialize(*primaryRay);
                    // BOUNCES PER RAY
                    // store primary ray depth information
                    primaryRay->getHitObject()->getNormal(*primaryRay); // update normal vector
                    float lambertCosineLaw = primaryRay->getNormal().dot(primaryRay->getDir()); // dot product of object normal and ray direction
                    lum = primaryRay->getHitObject()->getLum();
                    col = primaryRay->getHitObject()->getCol();
                    depthRed[0][0] = lum[0]; // object brightness R
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

                    for (int currentBounce = 1; currentBounce <= config.bounceDepth; currentBounce++) {
                        float randomSample = dist(rng); // monte carlo sampling
                        if (randomSample >= primaryRay->getHitObject()->getTransp()) {
                            sampleReflectionDirection(*nthRay, *primaryRay->getHitObject(), false);
                        } else {
                            sampleRefractionDirection(*nthRay, *primaryRay->getHitObject(), false);
                        }
                        BVHNode *leafNode = rootNode.searchBVHTree(*nthRay);
                        if (leafNode != nullptr) {
                            // did we make it to a leaf node?
                            SceneObject *BVHSceneObject = leafNode->getSceneObject();
                            std::vector<float> objectDistance = leafNode->getIntersectionDistance(*nthRay);
                            float distanceClose = objectDistance[0];
                            float distanceFar = objectDistance[1];
                            float distance = distanceClose;
                            nthRay->march(distanceClose - 0.05f); // march the ray to the objects bounding volume
                            while (distance <= distanceFar && !primaryRay->getHit()) {
                                nthRay->march(distance);
                                if (BVHSceneObject->intersectionCheck(*nthRay)) {
                                    nthRay->getHitPoint().set(nthRay->getPos());
                                    nthRay->setHit(true);
                                    nthRay->setHitObject(BVHSceneObject);
                                    // store hit data
                                    BVHSceneObject->getNormal(*nthRay); // update normal vector
                                    float lambertCosineLaw = nthRay->getNormal().dot(nthRay->getDir());
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
                                distance += config.secondaryRayStep;
                            }
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
                            greenAmplitude = ((depthGreen[index][0] +  greenAmplitude) * depthGreen[index][1]) * depthGreen[index][2];
                        }
                        if (depthBlue[index][3] == 1) {
                            blueAmplitude = ((depthBlue[index][0] + blueAmplitude) * depthBlue[index][1]) * depthBlue[index][2];
                        }
                    }
                    //absR[x][y] += redAmplitude;
                    //absG[x][y] += greenAmplitude;
                    //absB[x][y] += blueAmplitude;
                    //lumR[x][y] = absR[x][y] / static_cast<float>(currentRay);
                    //lumG[x][y] = absG[x][y] / static_cast<float>(currentRay);
                    //lumB[x][y] = absB[x][y] / static_cast<float>(currentRay);
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
            ray.updateOrigin(-0.1); // undo the march from the previous method
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
        ray.updateOrigin(sceneObject.getIntersectionDistance(ray)[1] + 0.1f);
    }
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
    gamma = 2 * pi * gamma; // azimuthal angle

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
    if (std::abs(ray.getNormal().getX() > 0.0001) || std::abs(ray.getNormal().getZ() > 0.0001)) {
        arbitraryA.set(0, 1, 0);
    } else { arbitraryA.set(1, 0, 0); }
    // tangent vector T equals cross product of normal N and arbitrary vector a
    Vector3 tangentT(arbitraryA.cross(ray.getNormal()));
    tangentT.normalise();

    // bitangnet vector B equals cross product of tangent and normal
    Vector3 bitangent(tangentT.cross(ray.getNormal()));
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
    ray.updateOrigin(0.1); // march the ray a tiny amount to move it off the object
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
                                                std::ref(BVHNodes), i, end, std::ref(bestCost), std::ref(indexLeft), std::ref(indexRight), std::ref(bestLeft),
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

void Render::findBestPair(const std::vector<BVHNode *> &nodes, int start, int end, std::atomic<float> &globalBestCost, int &leftIndex, int &rightIndex,BVHNode *&bestLeft, BVHNode *&bestRight, std::mutex &mutex) {
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
    Ray ray1(Vector3(0.1, 0.1, 0.1), Vector3(1, 0, 0.1));
    ray1.getDir().normalise();
    bool hit = true;
    std::cout << "Searching BVH" << std::endl;
    auto startTime = std::chrono::high_resolution_clock::now();
    BVHNode *leafNode = BVHNodes.at(0)->searchBVHTree(ray1);
    if (leafNode != nullptr) {
        leafNode->getSceneObject()->printType();
        hit = leafNode->getSceneObject()->objectCulling(ray1);
    } else { hit = false; }
    auto durationTime = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - startTime);
    std::cout << "Finished tree traversal: " << durationTime.count() << "ns" << std::endl;
    std::cout << "Hit: " << hit << std::endl;

    Ray ray2(Vector3(0.1, 0.1, 0.1), Vector3(1, 0, 0.1));
    ray2.getDir().normalise();
    hit = true;
    std::cout << "Searching BVH" << std::endl;
    startTime = std::chrono::high_resolution_clock::now();
    leafNode = BVHNodes.at(0)->searchBVHTree(ray2);
    if (leafNode != nullptr) {
        leafNode->getSceneObject()->printType();
        hit = leafNode->getSceneObject()->objectCulling(ray2);
    } else { hit = false; }
    durationTime = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - startTime);
    std::cout << "Finished tree traversal: " << durationTime.count() << "ns" << std::endl;
    std::cout << "Hit: " << hit << std::endl;

}

void Render::intialiseObjects() {
    for (int i = 0; i < resX; ++i) {
        for (int j = 0; j < resY; ++j) {
            primaryRay[i][j] = new Ray(); // allocate new Ray
            secondaryRay[i][j] = new Ray();
        }
    }
}

void Render::deleteObjects() {
    for (int i = 0; i < resX; ++i) {
        for (int j = 0; j < resY; ++j) {
            delete primaryRay[i][j];
            delete secondaryRay[i][j];
        }
    }
    for (auto node: BVHNodes) {
        delete node;
    }
    BVHNodes.clear();
}

std::pair<int, int> Render::secondarySegments(float resInput, int &numThreads, std::pair<int, int>, int step) {
    int res = resInput;
    int pixelsPerThread = res / numThreads; // number of pixels per thread
    int remainder = res % numThreads; // remaining pixels after division

    int start = step * pixelsPerThread + std::min(step, remainder);
    int end = (step + 1) * pixelsPerThread + std::min(step + 1, remainder) - 1;
    return std::make_pair(start, end);
}
