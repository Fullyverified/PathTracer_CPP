#include "BVHNode.h"

#include <unordered_map>

#include "BoundingBox.h"
#include "SceneObject.h"

// leaf node
BVHNode::BVHNode(BoundingBox* boundingBox, SceneObject& sceneObject) :
boundingBox(boundingBox), sceneObject(&sceneObject),
nodeLeft(nullptr), nodeRight(nullptr){}

// other node
BVHNode::BVHNode(BoundingBox* boundingBox, BVHNode* left, BVHNode* right) :
boundingBox(boundingBox), sceneObject(nullptr),
nodeLeft(left), nodeRight(right){}

// deconstructor
BVHNode::~BVHNode() {
    if (nodeLeft) {
        delete nodeLeft;
        nodeLeft = nullptr;
    }
    if (nodeRight) {
        delete nodeRight;
        nodeRight;
    }
    if (boundingBox) {
        delete boundingBox;
        boundingBox = nullptr;
    }
}

int BVHNode::getNumChildren() const {
    if (sceneObject != nullptr) {
        return 1;
    }
    int leftChildren = (nodeLeft != nullptr) ? nodeLeft->getNumChildren() : 0;
    int rightChildren = (nodeLeft != nullptr) ? nodeRight->getNumChildren() : 0;
    return leftChildren + rightChildren;
}

std::vector<float> BVHNode::getIntersectionDistance(Ray &ray) const {
    return boundingBox->getIntersectionDistance(ray);
}

float BVHNode::getArea() const {
    return boundingBox->getArea();
}

BoundingBox* BVHNode::getBoundingBox() const {
    return boundingBox;
}

SceneObject* BVHNode::getSceneObject() const {
    return sceneObject;
}

BVHNode* BVHNode::getNodeLeft() const {
    return nodeLeft;
}

BVHNode* BVHNode::getNodeRight() const {
    return nodeRight;
}

BVHNode* BVHNode::searchBVHTreeRoot(Ray &ray) {
    std::unordered_map<BVHNode*, traversalData> cache;

    return searchBVHTreeRecursive(ray, cache);
}

BVHNode* BVHNode::searchBVHTreeRecursive(Ray &ray, std::unordered_map<BVHNode*, traversalData>& cache) {

    auto iterator = cache.find(this);
    if (iterator != cache.end()) {
        return iterator->second.node;  // Use the cached node
    } else {
        // compute intersection distances and add to the cache
        std::vector<float> bboxDistance = boundingBox->getIntersectionDistance(ray);
        std::vector<float> objDistance = sceneObject ? sceneObject->getIntersectionDistance(ray) : std::vector<float> {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};

        traversalData data;
        data.bboxDistance = bboxDistance;
        data.objDistance = objDistance;
        data.node = this;
        cache[this] = data;
    }

    std::vector<float> bboxDistance = cache[this].bboxDistance;
    if (!(bboxDistance[0] <= bboxDistance[1] && bboxDistance[1] >= 0)) {
        //std::cout << "returning nullptr"<<std::endl;
        return nullptr; // ray does not intersect at all
    }

    if (sceneObject != nullptr) { // only return the node if the ray actually points at the object itself
        //std::cout << "Returning this"<<std::endl;
        std::vector<float> distanceObj = cache[this].objDistance;
        if (distanceObj[0] <= distanceObj[1] && distanceObj[1] >= 0) {
            return this;
        }
    }

    //std::cout <<"Determining left and right nodes"<<std::endl;
    BVHNode* hitLeft = nodeLeft == nullptr ? nullptr : nodeLeft->searchBVHTreeRecursive(ray, cache);
    BVHNode* hitRight = nodeRight == nullptr ? nullptr : nodeRight->searchBVHTreeRecursive(ray, cache);

    if (hitLeft != nullptr && hitRight != nullptr) {

        if (hitLeft->getSceneObject() != nullptr && hitRight->getSceneObject() != nullptr) {
            float distanceLeft = cache[hitLeft].objDistance[0];
            float distanceRight = cache[hitRight].objDistance[0];
            //std::cout <<"Returning closest Obj"<<std::endl;
            return distanceLeft < distanceRight ? hitLeft : hitRight;
        }

        if (hitLeft->getSceneObject() != nullptr) {
            float distanceLeft = cache[hitLeft].objDistance[0];
            float distanceRight = cache[hitRight].bboxDistance[0];
            //std::cout <<"Left Obj not null"<<std::endl;
            return distanceLeft < distanceRight ? hitLeft : hitRight;
        }

        if (hitRight->getSceneObject() != nullptr) {
            float distanceLeft = cache[hitLeft].bboxDistance[0];
            float distanceRight = cache[hitRight].objDistance[0];
            //std::cout <<"Right obj not null"<<std::endl;
            return distanceLeft < distanceRight ? hitLeft : hitRight;
        }

        float distanceLeft = cache[hitLeft].bboxDistance[0];
        float distanceRight = cache[hitRight].bboxDistance[0];
        if (distanceLeft < 0 && distanceRight < 0) { // both objects are behind the ray
            //std::cout <<"DistanceLeft < 0 && DistanceRight < 0"<<std::endl;
            return nullptr;
        }
        if (distanceLeft < 0) {
            //std::cout <<"DistanceLeft < 0 - return right"<<std::endl;
            return hitRight;
        }
        if (distanceRight < 0) {
            //std::cout <<"DistanceRight < 0 - return left"<<std::endl;
            return hitLeft;
        }
        //std::cout <<"Returning closest"<<std::endl;
        return distanceLeft < distanceRight ? hitLeft : hitRight;
    }

    if (hitLeft == nullptr && hitRight == nullptr) {
        //std::cout <<"Both null"<<std::endl;
        return nullptr;
    }

    //std::cout <<"One null, returning other"<<std::endl;
    return (hitLeft != nullptr) ? hitLeft : hitRight;
}

BVHNode* BVHNode::searchBVHTreeOLD(Ray &ray) {

    std::vector<float> distance = boundingBox->getIntersectionDistance(ray);
    if (!(distance[0] <= distance[1] && distance[1] >= 0)) {
        //std::cout << "returning nullptr"<<std::endl;
        return nullptr; // ray does not intersect at all
    }

    if (sceneObject != nullptr) { // only return the node if the ray actually points at the object itself
        //std::cout << "Returning this"<<std::endl;
        std::vector<float> distanceObj = sceneObject->getIntersectionDistance(ray);
        if (distanceObj[0] <= distanceObj[1] && distanceObj[1] >= 0) {
            return this;
        }
    }

    //std::cout <<"Determining left and right nodes"<<std::endl;
    BVHNode* hitLeft = nodeLeft == nullptr ? nullptr : nodeLeft->searchBVHTreeOLD(ray);
    BVHNode* hitRight = nodeRight == nullptr ? nullptr : nodeRight->searchBVHTreeOLD(ray);

    if (hitLeft != nullptr && hitRight != nullptr) {

        if (hitLeft->getSceneObject() != nullptr && hitRight->getSceneObject() != nullptr) {
            float distanceLeft = hitLeft->getSceneObject()->getIntersectionDistance(ray)[0];
            float distanceRight = hitRight->getSceneObject()->getIntersectionDistance(ray)[0];
            //std::cout <<"Returning closest Obj"<<std::endl;
            return distanceLeft < distanceRight ? hitLeft : hitRight;
        }

        /*if (hitLeft->getSceneObject() != nullptr) {
            float distanceLeft = hitLeft->getSceneObject()->getIntersectionDistance(ray)[0];
            float distanceRight = hitRight->getIntersectionDistance(ray)[0];
            //std::cout <<"Left Obj not null"<<std::endl;
            return distanceLeft < distanceRight ? hitLeft : hitRight;
        }

        if (hitRight->getSceneObject() != nullptr) {
            float distanceLeft = hitLeft->getIntersectionDistance(ray)[0];
            float distanceRight = hitRight->getSceneObject()->getIntersectionDistance(ray)[0];
            //std::cout <<"Right obj not null"<<std::endl;
            return distanceLeft < distanceRight ? hitLeft : hitRight;
        }*/

        float distanceLeft = hitLeft->getIntersectionDistance(ray)[0];
        float distanceRight = hitRight->getIntersectionDistance(ray)[0];
        if (distanceLeft < 0 && distanceRight < 0) { // both objects are behind the ray
            //std::cout <<"DistanceLeft < 0 && DistanceRight < 0"<<std::endl;
            return nullptr;
        }
        if (distanceLeft < 0) {
            //std::cout <<"DistanceLeft < 0 - return right"<<std::endl;
            return hitRight;
        }
        if (distanceRight < 0) {
            //std::cout <<"DistanceRight < 0 - return left"<<std::endl;
            return hitLeft;
        }
        //std::cout <<"Returning closest"<<std::endl;
        return distanceLeft < distanceRight ? hitLeft : hitRight;
    }

    if (hitLeft == nullptr && hitRight == nullptr) {
        //std::cout <<"Both null"<<std::endl;
        return nullptr;
    }

    //std::cout <<"One null, returning other"<<std::endl;
    return (hitLeft != nullptr) ? hitLeft : hitRight;
}

/*BVHNode* BVHNode::searchBVHTreeOLD(Ray &ray) {
    if (!boundingBox->objectCulling(ray)) {
        //std::cout << "returning nullptr"<<std::endl;
        return nullptr; // ray does not intersect at all
    }

    if (sceneObject != nullptr) {
        //sceneObject->printType();
    }
    if (sceneObject != nullptr && sceneObject->objectCulling(ray)) { // only return the node if the ray actually points at the object itself
        //std::cout << "Returning this"<<std::endl;
        return this;
    }

    //std::cout <<"Determining left and right nodes"<<std::endl;
    BVHNode* hitLeft = nodeLeft == nullptr ? nullptr : nodeLeft->searchBVHTreeOLD(ray);
    BVHNode* hitRight = nodeRight == nullptr ? nullptr : nodeRight->searchBVHTreeOLD(ray);

    if (hitLeft != nullptr && hitRight != nullptr) {
        float distanceLeft = hitLeft->getIntersectionDistance(ray)[0];
        float distanceRight = hitRight->getIntersectionDistance(ray)[0];

        if (distanceLeft < 0 && distanceRight < 0) { // both objects are behind the ray
            //std::cout <<"DistanceLeft < 0 && DistanceRight < 0"<<std::endl;
            return nullptr;
        }
        if (distanceLeft < 0) {
            //std::cout <<"DistanceLeft < 0 - return right"<<std::endl;
            return hitRight;
        }
        if (distanceRight < 0) {
            //std::cout <<"DistanceRight < 0 - return left"<<std::endl;
            return hitLeft;
        }
        //std::cout <<"Returning closest"<<std::endl;
        return distanceLeft < distanceRight ? hitLeft : hitRight;
    }

    if (hitLeft == nullptr && hitRight == nullptr) {
        //std::cout <<"Both null"<<std::endl;
        return nullptr;
    }

    //std::cout <<"One null, returning other"<<std::endl;
    return (hitLeft != nullptr) ? hitLeft : hitRight;
}*/