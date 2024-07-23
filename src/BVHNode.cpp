#include <BVHNode.h>
#include <Boundingbox.h>

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

SceneObject* BVHNode::getSceneObject() const {
    return sceneObject;
}

BVHNode* BVHNode::getNodeLeft() const {
    return nodeLeft;
}

BVHNode* BVHNode::getNodeRight() const {
    return nodeRight;
}

BVHNode* BVHNode::searchBVHTree(const Ray &ray) {
    //std::cout<<"Checking if Ray intersects BoundingBox of node"<<std::endl;
    if (!boundingBox->objectCulling(ray)) {
        return nullptr; // ray does not intersect at all
    }
    //std::cout<<"Checking if this node is a leaf node"<<std::endl;
    if (sceneObject != nullptr && sceneObject->objectCulling(ray)) {
        return this;
    }

    //std::cout<<"Recursive search left"<<std::endl;
    BVHNode* hitLeft = nodeLeft == nullptr ? nullptr : nodeLeft->searchBVHTree(ray);
    //std::cout<<"Recursive search right"<<std::endl;
    BVHNode* hitRight = nodeRight == nullptr ? nullptr : nodeRight->searchBVHTree(ray);

    //std::cout<<"Returning left or right node for whichever one the ray hits"<<std::endl;
    if (hitLeft != nullptr && hitRight != nullptr) {
        return hitLeft->getIntersectionDistance(ray) < hitRight->getIntersectionDistance(ray) ? hitLeft : hitRight;
    }

    //std::cout<<"Returning null if ray hits neither node"<<std::endl;
    if (hitLeft == nullptr && hitRight == nullptr) {
        return nullptr;
    }

    //std::cout<<"Returning the other node, of whichever is null"<<std::endl;
    return (hitLeft != nullptr) ? hitLeft : hitRight;
}