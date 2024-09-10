#include "BVHNode.h"

#include "BoundingBox.h"
#include "SceneObject.h"

// leaf node
BVHNode::BVHNode(BoundingBox *boundingBox, SceneObject &sceneObject) : boundingBox(boundingBox), sceneObject(&sceneObject),
                                                                       nodeLeft(nullptr), nodeRight(nullptr), isLeaf(false) {
}

// other node
BVHNode::BVHNode(BoundingBox *boundingBox, BVHNode *left, BVHNode *right) : boundingBox(boundingBox), sceneObject(nullptr),
                                                                            nodeLeft(left), nodeRight(right), isLeaf(false) {
}

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

std::pair<float, float> BVHNode::getIntersectionDistance(Ray &ray) const {
    return boundingBox->getIntersectionDistance(ray);
}

float BVHNode::getArea() const {
    return boundingBox->getArea();
}

BoundingBox *BVHNode::getBoundingBox() const {
    return boundingBox;
}

SceneObject *BVHNode::getSceneObject() const {
    return sceneObject;
}

BVHNode *BVHNode::getNodeLeft() const {
    return nodeLeft;
}

BVHNode *BVHNode::getNodeRight() const {
    return nodeRight;
}

std::pair<BVHNode*, float> BVHNode::searchBVHTree(Ray &ray) {
    std::pair<float, float> distance = boundingBox->getIntersectionDistance(ray);
    if (!(distance.first <= distance.second && distance.second >= 0)) {
        //std::cout << "returning nullptr"<<std::endl;
        return {nullptr, -1}; // ray does not intersect at all
    }

    if (sceneObject != nullptr) { // only return the node if the ray actually points at the object itself
        //std::cout << "Returning this"<<std::endl;
        std::pair<float, float> distanceObj = sceneObject->getIntersectionDistance(ray);
        if (distanceObj.first <= distanceObj.second && distanceObj.second >= 0) {
            return {this, distanceObj.first};
        }
    }

    //std::cout <<"Determining left and right nodes"<<std::endl;
    std::pair<BVHNode *, float> hitLeft = nodeLeft == nullptr ? std::make_pair(nullptr, -1.0f) : nodeLeft->searchBVHTree(ray);
    std::pair<BVHNode *, float> hitRight = nodeRight == nullptr ? std::make_pair(nullptr, -1.0f) : nodeRight->searchBVHTree(ray);

    if (hitLeft.first != nullptr && hitRight.first != nullptr) { // both valid nodes
        //std::cout <<"Returning closest Obj"<<std::endl;
        return hitLeft.second < hitRight.second ? hitLeft : hitRight;
    }

    if (hitLeft.first == nullptr && hitRight.first == nullptr) { // both nullptr
        //std::cout <<"Both null"<<std::endl;
        return {nullptr, -1.0f};
    }

    //std::cout <<"One null, returning other"<<std::endl;
    return (hitLeft.first != nullptr) ? hitLeft : hitRight; // one is null
}

void BVHNode::setLeaf(bool leaf) { isLeaf = leaf; }

[[nodiscard]] bool BVHNode::getLeaf() { return isLeaf; }
