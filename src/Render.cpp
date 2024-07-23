#include "Render.h"

#include <atomic>
#include <algorithm>
#include <chrono>
#include <future>
#include <iostream>
#include <thread>
#include <mutex>
#include <vector>

Render::Render() = default;

void Render::traverseBVH() {
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
    std::cout << "Finished tree traversal: " << durationTime << std::endl;
    std::cout << "Hit: " << hit << std::endl;
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

        for (int i = 0; i < BVHNodes.size(); i++) {
            for (int j = i + 1; j < BVHNodes.size(); j++) {
                BoundingBox *combinedBox = new BoundingBox(*BVHNodes.at(i)->getBoundingBox(),
                                                           *BVHNodes.at(j)->getBoundingBox());
                cost = (combinedBox->getArea() / (BVHNodes.at(i)->getArea() + BVHNodes.at(j)->getArea())) * (BVHNodes.at(i)->getNumChildren() + BVHNodes.at(j)->getNumChildren());

                if (cost < bestCost) {
                    bestCost = cost;
                    bestLeft = BVHNodes.at(i);
                    bestRight = BVHNodes.at(j);
                    indexLeft = i;
                    indexRight = j;
                }
                delete combinedBox;
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
    std::cout << "Finished tree creation: " << durationTime << std::endl;
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
    std::vector<std::thread> threads;
    //std::vector<std::future<void>> threads;
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
                threads.emplace_back(&Render::findBestPair, this, std::ref(BVHNodes), start, end, std::ref(bestCost),
                                     std::ref(indexLeft), std::ref(indexRight), std::ref(bestLeft), std::ref(bestRight),
                                     std::ref(mutex));
                //threads.emplace_back(std::async(std::launch::async, &Render::findBestPair, this, std::ref(BVHNodes), start, end, std::ref(bestCost), std::ref(indexLeft), std::ref(indexRight), std::ref(bestLeft), std::ref(bestRight), std::ref(mutex)));
            }
        } else {
            for (int i = 0; i < BVHNodes.size(); i++) {
                int end = i + 1;
                threads.emplace_back(&Render::findBestPair, this, std::ref(BVHNodes), i, end, std::ref(bestCost),
                                     std::ref(indexLeft), std::ref(indexRight), std::ref(bestLeft), std::ref(bestRight),
                                     std::ref(mutex));
                //threads.emplace_back(std::async(std::launch::async, &Render::findBestPair, this, std::ref(BVHNodes), i, end, std::ref(bestCost), std::ref(indexLeft), std::ref(indexRight), std::ref(bestLeft), std::ref(bestRight), std::ref(mutex)));
            }
        }

        for (std::thread &thread: threads) {
            thread.join();
        }
        threads.clear();
        /*for (std::future<void> &thread : threads) {
            thread.get(); // Blocks until the thread completes its task
        }
        threads.clear();*/

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
    std::cout << "Finished tree creation: " << durationTime << std::endl;
    std::cout << "BVHNode size: " << BVHNodes.size() << std::endl;
    std::cout << "RootNode numChildren: " << BVHNodes.at(0)->getNumChildren() << std::endl;
}

void Render::findBestPair(const std::vector<BVHNode *> &nodes, int start, int end, std::atomic<float> &globalBestCost,
                          int &leftIndex, int &rightIndex, BVHNode *&bestLeft, BVHNode *&bestRight, std::mutex &mutex) {
    float localBestCost = std::numeric_limits<float>::infinity();
    BVHNode *localBestLeft = nullptr, *localBestRight = nullptr;
    int localIndexLeft = 0, localIndexRight = 0;

    for (int i = start; i < end; i++) {
        for (int j = i + 1; j < nodes.size(); j++) {
            BoundingBox *combinedBox = new BoundingBox(*nodes[i]->getBoundingBox(), *nodes[j]->getBoundingBox());
            float cost = (combinedBox->getArea() / (nodes[i]->getArea() + nodes[j]->getArea())) * (
                             nodes[i]->getNumChildren() + nodes[j]->getNumChildren());
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
