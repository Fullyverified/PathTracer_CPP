#include <BVHNode.h>
#include <Boundingbox.h>

// leaf node
BVHNode::BVHNode(BoundingBox *boundingBox, SceneObject& sceneObject) :
boundingBox(boundingBox), sceneObject(&sceneObject),
nodeLeft(nullptr), nodeRight(nullptr){}

// other node
BVHNode::BVHNode(BoundingBox* boundingBox, BVHNode* left, BVHNode* right) :
boundingBox(boundingBox), sceneObject(nullptr),
nodeLeft(left), nodeRight(right){}

// deconstructor
BVHNode::~BVHNode() {
    delete nodeLeft;
    delete nodeRight;
    delete boundingBox;
    delete sceneObject;
}

int BVHNode::getNumChildren() const {

    if (sceneObject != nullptr) {
        return 1;
    }
    int leftChildren = (nodeLeft != nullptr) ? nodeLeft->getNumChildren() : 0;
    int rightChildren = (nodeLeft != nullptr) ? nodeRight->getNumChildren() : 0;
    return leftChildren + rightChildren;
}

float BVHNode::getIntersectionDistance(const Ray &ray) const {
    return boundingBox->getIntersectionDistance(ray);
}

float BVHNode::getArea() const {
    return boundingBox->getArea();
}

BoundingBox* BVHNode::getBoundingBox() const {
    return boundingBox;
}



BVHNode* BVHNode::searchBVHTree(const Ray &ray) {
    if (!boundingBox->objectCulling(ray)) {
        return nullptr; // ray does not intersect at all
    }

    if (sceneObject != nullptr && sceneObject->objectCulling(ray)) {
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