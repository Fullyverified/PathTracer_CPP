#include "BVHNode.h"

#include "BoundingBox.h"
#include "MeshObject.h"
#include "SceneObject.h"
#include "Matrix4x4.h"

// leaf node for scene objects
BVHNode::BVHNode(BoundingBox *boundingBox, SceneObject &sceneObject) : boundingBox(boundingBox), sceneObject(&sceneObject),
                                                                       nodeLeft(nullptr), nodeRight(nullptr), isLeaf(true) {
}

// tree node for scene objects
BVHNode::BVHNode(BoundingBox *boundingBox, BVHNode *left, BVHNode *right) : boundingBox(boundingBox), sceneObject(nullptr),
                                                                            nodeLeft(left), nodeRight(right), isLeaf(false) {
}

// node for triangles
BVHNode::BVHNode(BoundingBox *boundingBox, std::vector<Triangle *> triangles) : boundingBox(boundingBox), sceneObject(nullptr),
                                                                                triangles(triangles),
                                                                                nodeLeft(nullptr), nodeRight(nullptr), isLeaf(false) {
}

// deconstructor
BVHNode::~BVHNode() {
    if (nodeLeft) {
        delete nodeLeft;
        nodeLeft = nullptr;
    }
    if (nodeRight) {
        delete nodeRight;
        nodeRight = nullptr;
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

BVHNode::BVHResult BVHNode::searchBVHTreeScene(Ray &ray) {

    if (!isLeaf) {
        std::pair<float, float> bboxDistance = boundingBox->getIntersectionDistance(ray); // check node bounding box
        if (!(bboxDistance.first <= bboxDistance.second && bboxDistance.second >= 0)) {
            return {nullptr, -1.0f, -1.0f, nullptr}; // ray does not intersect at all
        }
    }
    // only return the node if the ray actually points at the object itself

    if (isLeaf) {
        if (!sceneObject->isMesh()) { // skip final bounding box for primatives
            Intersection result = sceneObject->getIntersectionDistance(ray);
            if (result.close <= result.far && result.far >= 0) {
                return {this, result.close, result.far, nullptr, {0}};
            }
            return {nullptr, -1.0f, -1.0f}; // early exit
        }


        std::pair<float, float> bboxDistance = boundingBox->getIntersectionDistance(ray); // check node bounding box for mesh objects
        if (!(bboxDistance.first <= bboxDistance.second && bboxDistance.second >= 0)) {
            return {nullptr, -1.0f, -1.0f, nullptr}; // ray does not intersect at all
        }
        if (sceneObject->isMesh()) {
            //Vector3 rayOriginOS = sceneObject->getInvTransform().MultiplyPoint(ray.getPos());
            //Vector3 rayDirOS = sceneObject->getInvTransform().MultiplyVector(ray.getDir());
            // transform to object space
            Ray rayOS(ray.getPos() + sceneObject->getPos(), ray.getDir());
            Intersection result = sceneObject->getIntersectionDistance(rayOS);
            if (result.close >= 0) {
                return {this, result.close, result.far, result.triangle, result.bCoords};
            }
            return {nullptr, -1.0f, -1.0f}; // early exit
        }
    }

    BVHResult hitLeft = nodeLeft == nullptr ? BVHResult(nullptr, -1.0f, -1.0f) : nodeLeft->searchBVHTreeScene(ray);
    //std::cout<<"Recursive search left"<<std::endl;
    BVHResult hitRight = nodeRight == nullptr ? BVHResult(nullptr, -1.0f, -1.0f) : nodeRight->searchBVHTreeScene(ray);
    //std::cout<<"Recursive search right"<<std::endl;

    if (hitLeft.node != nullptr && hitRight.node != nullptr) {
        // both valid nodes
        return hitLeft.close < hitRight.close ? hitLeft : hitRight;
    }

    if (hitLeft.node == nullptr && hitRight.node == nullptr) {
        // both nullptr
        return {nullptr, -1.0f, -1.0f};
    }

    return (hitLeft.node != nullptr) ? hitLeft : hitRight; // one is null, return the other
}

MeshObject::meshIntersection BVHNode::searchBVHTreeMesh(Ray &ray, const MeshObject* meshObject) {

    //std::cout<<"Searching Mesh Tree"<<std::endl;

    if (!isLeaf) {
        std::pair<float, float> bboxDistance = boundingBox->getIntersectionDistance(ray);

        if (!(bboxDistance.first <= bboxDistance.second && bboxDistance.second >= 0)) {
            //std::cout << "returning nullptr"<<std::endl;
            return {nullptr, -1.0f, nullptr, {0}}; // ray does not intersect at all
        }
    }

    if (isLeaf) {
        // only return the node if the ray actually points at the object itself
        //std::cout << "Is leaf"<<std::endl;
        MeshObject::meshIntersection meshIntersection = meshObject->intersectTriangles(ray, this);
        if (meshIntersection.close != -1.0f) {
            return {this, meshIntersection.close, meshIntersection.triangle, meshIntersection.bcoords};
        }
        return {nullptr, -1.0f, nullptr, {0}}; // early exit
    }

    //std::cout <<"Determining left and right nodes"<<std::endl;
    MeshObject::meshIntersection hitLeft = nodeLeft == nullptr ? MeshObject::meshIntersection(nullptr, -1.0f, nullptr, {1.0f}) : nodeLeft->searchBVHTreeMesh(ray, meshObject);
    MeshObject::meshIntersection hitRight = nodeRight == nullptr ? MeshObject::meshIntersection(nullptr, -1.0f, nullptr, {1.0f}) : nodeRight->searchBVHTreeMesh(ray, meshObject);

    if (hitLeft.node != nullptr && hitRight.node != nullptr) {
        // both valid nodes
        //std::cout <<"Returning closest Obj"<<std::endl;
        return hitLeft.close < hitRight.close ? hitLeft : hitRight;
    }

    if (hitLeft.node == nullptr && hitRight.node == nullptr) {
        // both nullptr
        //std::cout <<"Both null"<<std::endl;
        return {nullptr, -1.0f, nullptr, {0}};
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

BoundingBox *BVHNode::getBoundingBox() const { return boundingBox; }
std::pair<Vector3, Vector3> BVHNode::getBounds() { return boundingBox->getBounds(); }
void BVHNode::setLeaf(bool isLeaf) { this->isLeaf = isLeaf; }
void BVHNode::setLeft(BVHNode *left) { this->nodeLeft = left; }
void BVHNode::setRight(BVHNode *right) { this->nodeRight = right; }