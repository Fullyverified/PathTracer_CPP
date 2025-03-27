#ifndef LOADMESH_H
#define LOADMESH_H

#include <iostream>
#include <string>

#include <vector>

#include "BoundingBox.h"
#include "Triangle.h"

class LoadMesh {
public:

    LoadMesh() : rootNode(nullptr), vertexNormals(false) {
    }

    void load(const std::string& filename);


    void computeBounds();
    void createBVH();
    void cleanBVH(BVHNode* node);
    void numChildren(BVHNode* node, int& numChildren);
    std::pair<std::vector<Triangle*>, std::vector<Triangle*>> partitionTriangles(std::vector<Triangle*> triangles, int axis);
    [[nodiscard]] std::pair<Vector3, Vector3> triangleBounds(std::vector<Triangle*> triangles);

    [[nodiscard]] std::vector<Triangle*>& getTriangles() {return triangles;}
    [[nodiscard]] BVHNode* getRootNode() {return rootNode;}
    [[nodiscard]] std::pair<Vector3, Vector3> getBounds();

    bool isVertexNormals();
    bool isTexCoords();

private:
    std::vector<Triangle*> triangles; // store the triangles
    Vector3 minBounds, maxBounds;
    BVHNode* rootNode;
    bool vertexNormals;
    bool texCoords;
};

#endif //LOADMESH_H
