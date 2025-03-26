#ifndef BVHNODE_H
#define BVHNODE_H

#include "BoundingBox.h"
#include "MeshObject.h"
#include "Triangle.h"

class SceneObject;

class BVHNode {
public:
    struct BVHResult {
        BVHNode* node = nullptr;
        float close = -1.0f;
        float far = -1.0f;
        Triangle* triangle = nullptr;
        Vector3 bCoords = {0};
    };

    // two constructors
    BVHNode(BoundingBox* boundingBox, SceneObject& sceneObject);
    BVHNode(BoundingBox* boundingBox, BVHNode* left, BVHNode* right);
    BVHNode(BoundingBox *boundingBox, std::vector<Triangle*> triangles);

    ~BVHNode(); // deconstructor

    [[nodiscard]] int getNumChildren() const;
    [[nodiscard]] BVHResult searchBVHTreeScene(Ray &ray);
    [[nodiscard]] MeshObject::meshIntersection searchBVHTreeMesh(Ray &ray, const MeshObject* meshObject);
    [[nodiscard]] float getArea() {return boundingBox->getArea();}
    std::pair<Vector3, Vector3> getBounds();

    [[nodiscard]] BoundingBox* getBoundingBox() const;
    [[nodiscard]] SceneObject* getSceneObject() const {return sceneObject;}
    [[nodiscard]] std::vector<Triangle*> getTriangles() {return triangles;}
    void setTriangles(std::vector<Triangle*> triangles) {this->triangles = triangles;}
    void setLeft(BVHNode* left);
    void setRight(BVHNode* right);
    [[nodiscard]] BVHNode* getNodeLeft() const {return nodeLeft;}
    [[nodiscard]] BVHNode* getNodeRight() const { return nodeRight;}

    void setLeaf(bool isLeaf);
    [[nodiscard]] bool getLeaf() {return isLeaf;}

    //void setTriangle(Triangle* triangle) {this->triangle = triangle;}
    //Triangle* getTriangle() {return triangle;}

private:
    BVHNode *nodeLeft, *nodeRight;
    SceneObject *sceneObject;
    std::vector<Triangle*> triangles;
    //Triangle* triangle;
    BoundingBox *boundingBox;
    bool isLeaf;
};

#endif //BVHNODE_H
