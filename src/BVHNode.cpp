#include "BVHNode.h"

#include "BoundingBox.h"
#include "MeshObject.h"
#include "SceneObject.h"

// leaf node for scene objects
BVHNode::BVHNode(BoundingBox *boundingBox, SceneObject &sceneObject) : boundingBox(boundingBox), sceneObject(&sceneObject),
                                                                       nodeLeft(nullptr), nodeRight(nullptr), isLeaf(true) {}

// tree node for scene objects
BVHNode::BVHNode(BoundingBox *boundingBox, BVHNode *left, BVHNode *right) : boundingBox(boundingBox), sceneObject(nullptr),
                                                                            nodeLeft(left), nodeRight(right), isLeaf(false) {}

// node for triangles
BVHNode::BVHNode(BoundingBox *boundingBox, std::vector<Triangle*> triangles, bool isLeaf) : boundingBox(boundingBox), sceneObject(nullptr),
                                                                                             triangles(triangles),
                                                                                             nodeLeft(nullptr), nodeRight(nullptr), isLeaf(isLeaf) {}
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
    if (!triangles.empty()) {
        for (Triangle *triangle: triangles) {
            delete triangle;
        }
    }
}

struct BVHNode::BVHResult BVHNode::searchBVHTreeScene(Ray &ray) {
    std::pair<float, float> bboxDistance = boundingBox->getIntersectionDistance(ray);
    if (!(bboxDistance.first <= bboxDistance.second && bboxDistance.second >= 0)) {
        //std::cout << "returning nullptr"<<std::endl;
        return {nullptr, -1}; // ray does not intersect at all
    }

    if (sceneObject != nullptr) { // only return the node if the ray actually points at the object itself
        //std::cout << "Returning this"<<std::endl;
        if (!sceneObject->isMesh()) {
            //std::cout << "Primative"<<std::endl;
            std::pair<float, float> distanceObj = sceneObject->getIntersectionDistance(ray);
            if (distanceObj.first <= distanceObj.second && distanceObj.second >= 0) {
                return {this, distanceObj.first};
            }
        }
        if (sceneObject->isMesh()) {
            //std::cout << "Mesh Object"<<std::endl;
            std::pair<float, float> distanceObj = sceneObject->getIntersectionDistance(ray);
            if (distanceObj.first <= distanceObj.second && distanceObj.second >= 0) {
                return {this, distanceObj.first};
            }
        }
    }

    //std::cout <<"Determining left and right nodes"<<std::endl;
    BVHResult hitLeft = nodeLeft == nullptr ? BVHResult(nullptr, -1.0f, -1.0f) : nodeLeft->searchBVHTreeScene(ray);
    BVHResult hitRight = nodeRight == nullptr ? BVHResult(nullptr, -1.0f, -1.0f) : nodeRight->searchBVHTreeScene(ray);

    if (hitLeft.node != nullptr && hitRight.node != nullptr) {
        // both valid nodes
        //std::cout <<"Returning closest Obj"<<std::endl;
        return hitLeft.close < hitRight.close ? hitLeft : hitRight;
    }

    if (hitLeft.node == nullptr && hitRight.node == nullptr) {
        // both nullptr
        //std::cout <<"Both null"<<std::endl;
        return {nullptr, -1.0f};
    }

    //std::cout <<"One null, returning other"<<std::endl;
    return (hitLeft.node != nullptr) ? hitLeft : hitRight; // one is null
}

struct BVHNode::BVHResult BVHNode::searchBVHTreeMesh(Ray &ray, MeshObject::Transform &transform) {
    //std::cout<<"Searching Mesh Tree"<<std::endl;

    transform.rayToObj(ray);
    std::pair<float, float> bboxDistance = boundingBox->getIntersectionDistance(ray);
    transform.rayToWorld(ray);

    if (!(bboxDistance.first <= bboxDistance.second && bboxDistance.second >= 0)) {
        //std::cout << "returning nullptr"<<std::endl;
        return {nullptr, -1.0f, 1.0f}; // ray does not intersect at all
    }

    if (isLeaf) {
        // only return the node if the ray actually points at the object itself
        //std::cout << "Returning this"<<std::endl;
        transform.rayToObj(ray);
        std::pair<float, float> distanceObj = transform.meshObject->intersectTriangles(ray, this);
        transform.rayToWorld(ray);
        if (distanceObj.first <= distanceObj.second && distanceObj.second >= 0) {
            return {this, distanceObj.first, distanceObj.second};
        }
    }
    //std::cout <<"Determining left and right nodes"<<std::endl;
    BVHResult hitLeft = nodeLeft == nullptr ? BVHResult(nullptr, -1.0f, -1.0f) : nodeLeft->searchBVHTreeMesh(ray, transform);
    BVHResult hitRight = nodeRight == nullptr ? BVHResult(nullptr, -1.0f, -1.0f) : nodeRight->searchBVHTreeMesh(ray, transform);

    if (hitLeft.node != nullptr && hitRight.node != nullptr) {
        // both valid nodes
        //std::cout <<"Returning closest Obj"<<std::endl;
        return hitLeft.close < hitRight.close ? hitLeft : hitRight;
    }

    if (hitLeft.node == nullptr && hitRight.node == nullptr) {
        // both nullptr
        //std::cout <<"Both null"<<std::endl;
        return {nullptr, -1.0f, -1.0f};
    }

    //std::cout <<"One null, returning other"<<std::endl;
    return (hitLeft.node != nullptr) ? hitLeft : hitRight; // one is null
}

int BVHNode::getNumChildren() const {
    if (sceneObject != nullptr) {
        return 1;
    }
    int leftChildren = (nodeLeft != nullptr) ? nodeLeft->getNumChildren() : 0;
    int rightChildren = (nodeLeft != nullptr) ? nodeRight->getNumChildren() : 0;
    return leftChildren + rightChildren;
}

float BVHNode::getArea() {
    return boundingBox->getArea();
}

BoundingBox *BVHNode::getBoundingBox() const { return boundingBox; }
SceneObject *BVHNode::getSceneObject() const { return sceneObject; }
std::vector<Triangle *> BVHNode::getTriangles() { return triangles; }
void BVHNode::setTriangles(std::vector<Triangle *> triangles) { this->triangles = triangles; }
BVHNode *BVHNode::getNodeLeft() const { return nodeLeft; }
BVHNode *BVHNode::getNodeRight() const { return nodeRight; }
std::pair<Vector3, Vector3> BVHNode::getBounds() { return boundingBox->getBounds(); }
void BVHNode::setLeaf(bool isLeaf) { this->isLeaf = isLeaf; }
[[nodiscard]] bool BVHNode::getLeaf() { return isLeaf; }
void BVHNode::setLeft(BVHNode *left) { this->nodeLeft = left; }
void BVHNode::setRight(BVHNode *right) { this->nodeRight = right; }
