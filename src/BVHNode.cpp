#include "BVHNode.h"
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

BVHNode* BVHNode::searchBVHTree(Ray &ray) {
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
    BVHNode* hitLeft = nodeLeft == nullptr ? nullptr : nodeLeft->searchBVHTree(ray);
    BVHNode* hitRight = nodeRight == nullptr ? nullptr : nodeRight->searchBVHTree(ray);

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
}