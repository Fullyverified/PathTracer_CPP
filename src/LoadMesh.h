#ifndef LOADMESH_H
#define LOADMESH_H

#include <iostream>
#include <string>

#include <vector>
#include "Triangle.h"

class LoadMesh {
public:

    LoadMesh() = default;

    void load(const std::string& filename);

    [[nodiscard]] std::vector<Triangle*>& getTriangles() {
        return triangles;
    }

    [[nodiscard]] std::pair<Vector3, Vector3> getBounds();

    void createBVH();

private:
    std::vector<Triangle*> triangles; // store the triangles
    Vector3 minBounds, maxBounds;
};

#endif //LOADMESH_H
