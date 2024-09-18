#ifndef BVHNODE_H
#define BVHNODE_H

#include "BoundingBox.h"
#include "MeshObject.h"
#include "SceneObject.h"
#include "Triangle.h"

class BVHNode {
public:
    struct BVHResult {
        BVHNode* node;
        float close;
        float far;
    };

    // two constructors
    BVHNode(BoundingBox* boundingBox, SceneObject& sceneObject);
    BVHNode(BoundingBox* boundingBox, BVHNode* left, BVHNode* right);
    BVHNode(BoundingBox *boundingBox, std::vector<Triangle*> triangles, bool isLeaf);

    ~BVHNode(); // deconstructor

    [[nodiscard]] int getNumChildren() const;
    [[nodiscard]] struct BVHResult searchBVHTreeScene(Ray &ray);
    [[nodiscard]] struct BVHResult searchBVHTreeMesh(Ray &ray, MeshObject::Transform &transform);
    [[nodiscard]] float getArea();
    std::pair<Vector3, Vector3> getBounds();

    [[nodiscard]] BoundingBox* getBoundingBox() const;
    [[nodiscard]] SceneObject* getSceneObject() const;
    [[nodiscard]] std::vector<Triangle*> getTriangles();
    void setTriangles(std::vector<Triangle*> triangles);
    void setLeft(BVHNode* left);
    void setRight(BVHNode* right);
    [[nodiscard]] BVHNode* getNodeLeft() const;
    [[nodiscard]] BVHNode* getNodeRight() const;

    void setLeaf(bool isLeaf);
    [[nodiscard]] bool getLeaf();

private:
    BVHNode *nodeLeft, *nodeRight;
    SceneObject *sceneObject;
    std::vector<Triangle*> triangles;
    BoundingBox *boundingBox;
    bool isLeaf;
};

#endif //BVHNODE_H
