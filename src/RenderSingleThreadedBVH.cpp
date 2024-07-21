#include "RenderSingleThreadedBVH.h"

#include <algorithm>
#include <iostream>

RenderSingleThreadedBVH::RenderSingleThreadedBVH() = default;

void RenderSingleThreadedBVH::constructBVH(const std::vector<SceneObject*> &sceneObjectsList) {
    for (int i = 0; i < sceneObjectsList.size(); i ++) {
        std::cout<<"i: " << i << std::endl;
        std::pair<Vector3, Vector3> bounds = sceneObjectsList[i]->getBounds();
        BoundingBox *boundingBox = new BoundingBox(bounds.first, bounds.second);
        std::cout << "Finished creating boundingBox" << std::endl;
        BVHNodes.emplace_back(new BVHNode(boundingBox, *sceneObjectsList[i]));
        std::cout << "Eplaced Node" << std::endl;
    }

    while (BVHNodes.size() > 1) {
        float cost = 0, bestCost = std::numeric_limits<float>::infinity();
        int indexLeft = 0, indexRight  = 0;
        BVHNode *bestLeft = nullptr;
        BVHNode *bestRight = nullptr;

        for (int i = 0; i < BVHNodes.size(); i++) {
            for (int j = i + 1; j < BVHNodes.size(); j++) {
                BoundingBox *combinedBox = new BoundingBox(*BVHNodes.at(i)->getBoundingBox(), *BVHNodes.at(j)->getBoundingBox());
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
        BoundingBox *parentBox = new BoundingBox(*bestLeft->boundingBox, *bestRight->boundingBox);
        BVHNode *parentNode = new BVHNode(parentBox, bestLeft, bestRight);
        BVHNodes.emplace_back(parentNode);
        // erase node from list
        if (indexLeft > indexRight) {
            BVHNodes.erase(BVHNodes.begin() + indexLeft);
            BVHNodes.erase(BVHNodes.begin() + indexLeft);
        }
        else {
            BVHNodes.erase(BVHNodes.begin() + indexLeft);
            BVHNodes.erase(BVHNodes.begin() + indexLeft);
        }
    }
    std::cout << "BVHNode size: " << BVHNodes.size() << std::endl;
    std::cout << "RootNode numChildren: " << BVHNodes.at(0)->getNumChildren() << std::endl;
}
