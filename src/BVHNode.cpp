#include <BVHNode.h>
#include <Boundingbox.h>
#include <SceneObject.h>

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
        return nullptr; // ray does not intersect at all
    }
    if (sceneObject != nullptr && sceneObject->objectCulling(ray)) { // only return the node if the ray actually points at the object itself
        return this;
    }

    BVHNode* hitLeft = nodeLeft == nullptr ? nullptr : nodeLeft->searchBVHTree(ray);
    BVHNode* hitRight = nodeRight == nullptr ? nullptr : nodeRight->searchBVHTree(ray);

    if (hitLeft != nullptr && hitRight != nullptr) {
        return hitLeft->getIntersectionDistance(ray) < hitRight->getIntersectionDistance(ray) ? hitLeft : hitRight;
    }

    if (hitLeft == nullptr && hitRight == nullptr) {
        return nullptr;
    }

    return (hitLeft != nullptr) ? hitLeft : hitRight;
}