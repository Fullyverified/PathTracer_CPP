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

Render::Render(const Config& config, const Camera& cam) : config(config), primaryRayStep(config.primaryRayStep), secondaryRayStep(config.secondaryRayStep), numRays(config.raysPerPixel),
numBounces(config.bouncesPerRay), frameTime(config.frameTime), cam(cam),
resX(cam.getResX()), resY(cam.getResY()) {
    primaryRay.resize(resX, std::vector<Ray*>(resY, nullptr));
    secondaryRay.resize(resX, std::vector<Ray*>(resY, nullptr));
    avgR.resize(resX, std::vector<float*>(resY, nullptr));
    avgG.resize(resX, std::vector<float*>(resY, nullptr));
    avgB.resize(resX, std::vector<float*>(resY, nullptr));
    absR.resize(resX, std::vector<float*>(resY, nullptr));
    absG.resize(resX, std::vector<float*>(resY, nullptr));
    absB.resize(resX, std::vector<float*>(resY, nullptr));
}

void Render::computePixels(std::vector<SceneObject*> &sceneobjectsList, Camera &cam) {
    int numThreads = std::thread::hardware_concurrency();
    //numThreads = 1;
    std::cout<<"Avaliable Threads: "<<numThreads<<std::endl;
    intialiseObjects(); // create all ray and luminance objects
    // construct BVH
    constructBVHST(sceneobjectsList);
    BVHNode *BVHrootNode = BVHNodes.at(0);

    auto startTime = std::chrono::high_resolution_clock::now();
    // primary rays
    std::mutex mutex;
    int x1, x2, y1, y2;
    std::pair<int, int> boundsX = std::pair(x1, x2);
    std::pair<int, int> boundsY = std::pair(y1, y2);

    std::vector<std::future<void>> threads;

    for (int j = 0; j < numThreads; j++) {
        for (int i = 0; i < numThreads; i++) {
            boundsX = threadedRenderSegmentation(cam.getResX(), numThreads, boundsX, i);
            boundsY = threadedRenderSegmentation(cam.getResY(), numThreads, boundsY, j);
            /*std::cout<<"Starting Thread:"<<std::endl;
            std::cout<<"X"<<boundsX.first<<"->"<<boundsX.second<<std::endl;
            std::cout<<"Y"<<boundsY.first<<"->"<<boundsY.second<<std::endl;*/
            threads.emplace_back(std::async(std::launch::async, &Render::computePrimaryRay, this, std::ref(cam), std::ref(primaryRay), boundsX.first, boundsX.second, boundsY.first, boundsY.second, std::ref(*BVHrootNode), std::ref(mutex)));
        }
    }

    //std::cout<<"Thread finished"<<std::endl;
    for (std::future<void> &thread : threads) {
        thread.get(); // Blocks until the thread completes its task
    }
    threads.clear();

    auto stopTime = std::chrono::high_resolution_clock::now();
    auto durationTime = std::chrono::duration_cast<std::chrono::milliseconds>(stopTime - startTime);
    std::cout<<"Multithreaded PrimaryRay Time: "<<durationTime.count()<<"ms"<<std::endl;

    computePrimaryRay(cam, primaryRay, 0, cam.getResX(), 0, cam.getResY(), *BVHrootNode, std::ref(mutex));

    stopTime = std::chrono::high_resolution_clock::now();
    durationTime = std::chrono::duration_cast<std::chrono::milliseconds>(stopTime - startTime);
    std::cout<<"Singlethreaded Primary Ray time: "<<durationTime.count()<<"ms"<<std::endl;

    std::cout<<"Deleting Objects"<<std::endl;
    deleteObjects(); // delete all ray and luminance vectors
}

void Render::computePrimaryRay(Camera &cam, std::vector<std::vector<Ray*>> &primaryRay, int xstart, int xend, int ystart, int yend, BVHNode &rootNode, std::mutex &mutex) const {
    for (int y = ystart; y < yend; y++) {
        for (int x = xstart; x < xend; x++) {
            Ray* ray = primaryRay[x][y];
            ray->getPos().setV(cam.getPos());
            // calculate position of pixel on image plane
            Vector3 pixelPosPlane(((x + 0.5f) / cam.getResX() * 2) - 1, 1 - ((x + 0.5f) / cam.getResX() * 2), 0);
            Vector3 pixelPosScene(pixelPosPlane.getX() * cam.getPlaneWidth(), pixelPosPlane.getY() * cam.getPlaneHeight(), 0);
            // point ray to pixel position (ray starts from camera origin)
            ray->getDir().set(
                cam.getDir().getX() + cam.getRight().getX() * pixelPosScene.getX() + cam.getUp().getX() * pixelPosScene.getY(),
                cam.getDir().getY() + cam.getRight().getY() * pixelPosScene.getX() + cam.getUp().getY() * pixelPosScene.getY(),
                cam.getDir().getZ() + cam.getRight().getZ() * pixelPosScene.getX() + cam.getUp().getZ() * pixelPosScene.getY());
            ray->getDir().normalise();
            ray->march(0);

            BVHNode *leafNode = rootNode.searchBVHTree(*ray);
            if (leafNode != nullptr) { // did we make it to a leaf node?
                SceneObject* BVHSceneObject = leafNode->getSceneObject();
                float distance = leafNode->getIntersectionDistance(*ray).first;
                float distanceFar = leafNode->getIntersectionDistance(*ray).second;
                ray->march(distance - 0.05f); // march the ray to the objects bounding volume
                while (distance <= distanceFar && !primaryRay[x][y]->getHit()) {
                    if (BVHSceneObject->intersectionCheck(*ray)) {
                        //ray->getHitPoint().setV(ray->getPos());
                        ray->setHit(true);
                        //ray.setHitObject(&BVHSceneObject);
                    }
                    distance += config.primaryRayStep;
                }
            }
            //std::cout<<"X: "<<x<<", Y: "<<y<<::std::endl;
        }
    }
}

void Render::computeSecondaryRay(Camera &cam, std::vector<std::vector<Ray>> &primaryRayV, std::vector<std::vector<Ray>> &secondaryRayV, BVHNode &rootNode) const {
    for (int y = 0; y < cam.getResY(); y++) {
        for (int x = 0; x < cam.getResX(); x++) {
            Ray primaryRay = primaryRayV[x][y];
        }
    }
}

void Render::cosineWeightedHemisphereImportanceSampling(Ray &ray, SceneObject *&sceneObject, bool flipNormal) {
}

void Render::refractionDirection(Ray &ray, SceneObject *&sceneObject) {
}

float Render::lambertCosineLaw(Ray &ray, SceneObject *sceneObject) {
    return 0;
}

void Render::constructBVHST(const std::vector<SceneObject*> &sceneObjectsList) {

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
    std::cout<<"Finished tree creation: "<<durationTime.count()<<"us"<<std::endl;
    //std::cout << "BVHNode size: " << BVHNodes.size() << std::endl;
    std::cout<<"RootNode numChildren: "<<BVHNodes.at(0)->getNumChildren() << std::endl;
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
    std::vector<std::future<void>> threads;
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
                //threads.emplace_back(&Render::findBestPair, this, std::ref(BVHNodes), start, end, std::ref(bestCost), std::ref(indexLeft), std::ref(indexRight), std::ref(bestLeft), std::ref(bestRight), std::ref(mutex));
                threads.emplace_back(std::async(std::launch::async, &Render::findBestPair, this, std::ref(BVHNodes), start, end, std::ref(bestCost), std::ref(indexLeft), std::ref(indexRight), std::ref(bestLeft), std::ref(bestRight), std::ref(mutex)));
            }
        } else {
            for (int i = 0; i < BVHNodes.size(); i++) {
                int end = i + 1;
                //threads.emplace_back(&Render::findBestPair, this, std::ref(BVHNodes), i, end, std::ref(bestCost), std::ref(indexLeft), std::ref(indexRight), std::ref(bestLeft), std::ref(bestRight), std::ref(mutex));
                threads.emplace_back(std::async(std::launch::async, &Render::findBestPair, this, std::ref(BVHNodes), i, end, std::ref(bestCost), std::ref(indexLeft), std::ref(indexRight), std::ref(bestLeft), std::ref(bestRight), std::ref(mutex)));
            }
        }

        /*for (std::thread &thread: threads) {
            thread.join();
        }
        threads.clear();*/
        for (std::future<void> &thread : threads) {
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
    std::cout << "Finished tree creation: "<<durationTime.count()<<"us"<<std::endl;
    std::cout << "BVHNode size: " << BVHNodes.size() << std::endl;
    std::cout << "RootNode numChildren: " << BVHNodes.at(0)->getNumChildren() << std::endl;
}

void Render::findBestPair(const std::vector<BVHNode *> &nodes, int start, int end, std::atomic<float> &globalBestCost,int &leftIndex, int &rightIndex, BVHNode *&bestLeft, BVHNode *&bestRight, std::mutex &mutex) {
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

void Render::BVHProilfing() {
    Ray ray1(Vector3(0.1, 0.1, 0.1), Vector3(1, 0.1, 0.1));
    ray1.getDir().normalise();
    bool hit = true;
    std::cout << "Searching BVH" << std::endl;
    auto startTime = std::chrono::high_resolution_clock::now();
    BVHNode *leafNode = BVHNodes.at(0)->searchBVHTree(ray1);
    if (leafNode != nullptr) {
        hit = leafNode->getSceneObject()->objectCulling(ray1);
    } else {hit = false;}
    auto stopTime = std::chrono::high_resolution_clock::now();
    auto durationTime = std::chrono::duration_cast<std::chrono::nanoseconds>(stopTime - startTime);
    std::cout << "Finished tree traversal: "<<durationTime.count()<<"ns"<<std::endl;
    std::cout << "Hit: " << hit << std::endl;
}

void Render::intialiseObjects() {
    for (int i = 0; i < resX; ++i) {
        for (int j = 0; j < resY; ++j) {
            primaryRay[i][j] = new Ray(Vector3(0,0,0)); // allocate new Ray
            secondaryRay[i][j] = new Ray(Vector3(0,0,0));
            avgR[i][j] = new float;
            avgG[i][j] = new float;
            avgB[i][j] = new float;
            absR[i][j] = new float;
            absG[i][j] = new float;
            absB[i][j] = new float;
        }
    }
}

void Render::deleteObjects() {
    for (int i = 0; i < resX; ++i) {
        for (int j = 0; j < resY; ++j) {
            delete primaryRay[i][j];
            delete secondaryRay[i][j];
            delete avgR[i][j];
            delete avgG[i][j];
            delete avgB[i][j];
            delete absR[i][j];
            delete absG[i][j];
            delete absB[i][j];
        }
    }
    for (auto node : BVHNodes) {
        delete node;
    }
    BVHNodes.clear();
}

std::pair<int, int> Render::threadedRenderSegmentation(float resI, int &numThreads, std::pair<int, int>, int i) {
    int res = resI;
    int pixelsPerThread = res / numThreads; // basic number of nodes per thread
    int remainder = res % numThreads; // remaining nodes after even distribution

    int start = i * pixelsPerThread + std::min(i, remainder);
    int end = start + pixelsPerThread + (i < remainder ? 1 : 0);
    end = std::min(end, res); // ensure last thread does not exceed size of nodes vector
    return std::make_pair(start, end);
}