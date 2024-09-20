#include "LoadMesh.h"
#include "config.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include <algorithm>
#include <queue>

#include "BVHNode.h"
#include "tiny_obj_loader.h"

void LoadMesh::load(const std::string &filename) {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;

    if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, filename.c_str())) {
        throw std::runtime_error(warn + err);
    }

    // Process vertices, normals, and faces
    for (const auto &shape: shapes) {
        for (size_t f = 0; f < shape.mesh.indices.size() / 3; f++) {
            tinyobj::index_t idx0 = shape.mesh.indices[3 * f + 0];
            tinyobj::index_t idx1 = shape.mesh.indices[3 * f + 1];
            tinyobj::index_t idx2 = shape.mesh.indices[3 * f + 2];

            // vertices
            const Vector3 v0(
                attrib.vertices[3 * idx0.vertex_index + 0],
                attrib.vertices[3 * idx0.vertex_index + 1],
                attrib.vertices[3 * idx0.vertex_index + 2]
            );
            const Vector3 v1(
                attrib.vertices[3 * idx1.vertex_index + 0],
                attrib.vertices[3 * idx1.vertex_index + 1],
                attrib.vertices[3 * idx1.vertex_index + 2]
            );
            const Vector3 v2(
                attrib.vertices[3 * idx2.vertex_index + 0],
                attrib.vertices[3 * idx2.vertex_index + 1],
                attrib.vertices[3 * idx2.vertex_index + 2]
            );

            // normals
            Vector3 n0(0, 0, 0), n1(0, 0, 0), n2(0, 0, 0);
            if (idx0.normal_index >= 0) {
                n0 = Vector3(
                    attrib.normals[3 * idx0.normal_index + 0],
                    attrib.normals[3 * idx0.normal_index + 1],
                    attrib.normals[3 * idx0.normal_index + 2]
                );
            }
            if (idx1.normal_index >= 0) {
                n1 = Vector3(
                    attrib.normals[3 * idx1.normal_index + 0],
                    attrib.normals[3 * idx1.normal_index + 1],
                    attrib.normals[3 * idx1.normal_index + 2]
                );
            }
            if (idx2.normal_index >= 0) {
                n2 = Vector3(
                    attrib.normals[3 * idx2.normal_index + 0],
                    attrib.normals[3 * idx2.normal_index + 1],
                    attrib.normals[3 * idx2.normal_index + 2]
                );
            }
            triangles.emplace_back(new Triangle(v0, v1, v2, n0, n1, n2));
        }
    }
    computeBounds();
    createBVH();
    cleanBVH(rootNode);
}

void LoadMesh::computeBounds() {
    minBounds.set(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
    maxBounds.set(-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity());

    for (const Triangle* triangle : triangles) {
        Vector3 vertices[] = {triangle->v0, triangle->v1, triangle->v2};

        for (const auto& vertex : vertices) {
            if (vertex.getX() < minBounds.getX()) minBounds.setX(vertex.getX());
            if (vertex.getY() < minBounds.getY()) minBounds.setY(vertex.getY());
            if (vertex.getZ() < minBounds.getZ()) minBounds.setZ(vertex.getZ());

            if (vertex.getX() > maxBounds.getX()) maxBounds.setX(vertex.getX());
            if (vertex.getY() > maxBounds.getY()) maxBounds.setY(vertex.getY());
            if (vertex.getZ() > maxBounds.getZ()) maxBounds.setZ(vertex.getZ());
        }
    }
}


std::pair<Vector3, Vector3> LoadMesh::getBounds() {
    return {minBounds, maxBounds};
}

void LoadMesh::createBVH() {
    // top down construction
    auto bounds = getBounds();
    BoundingBox* parentBox = new BoundingBox(bounds.first, bounds.second);
    rootNode = new BVHNode(parentBox, triangles);

    std::queue<BVHNode*> nodes;
    nodes.push(rootNode);

    while (!nodes.empty()) {
        BVHNode* currentNode = nodes.front();
        nodes.pop();

        std::vector<Triangle*> currentTriangles = currentNode->getTriangles();
        if (currentTriangles.size() <= config.trisPerNode) {
            currentNode->setLeaf(true);
            continue; // node is already a leaf node
        }

        auto currentBounds = currentNode->getBounds();
        // find split axis
        Vector3 size = currentBounds.second - currentBounds.first;
        int axis = 0;
        if (size.getY() >= size.getX() && size.getY() >= size.getZ()) {
            axis = 1;
        } else if (size.getZ() >= size.getX() && size.getZ() >= size.getY()) {
            axis = 2;
        }

        std::pair<std::vector<Triangle*>, std::vector<Triangle*>> subdividedTriangles = partitionTriangles(currentTriangles, axis);

        // create child nodes
        std::pair<Vector3, Vector3> boundsLeft = triangleBounds(subdividedTriangles.first);
        BoundingBox* leftBox = new BoundingBox(boundsLeft.first, boundsLeft.second);
        std::pair<Vector3, Vector3> boundsRight = triangleBounds(subdividedTriangles.second);
        BoundingBox* rightBox = new BoundingBox(boundsRight.first, boundsRight.second);

        BVHNode* leftChild = new BVHNode(leftBox, subdividedTriangles.first);
        BVHNode* rightChild = new BVHNode(rightBox,  subdividedTriangles.second);

        currentNode->setLeft(leftChild);
        currentNode->setRight(rightChild);

        nodes.push(leftChild);
        nodes.push(rightChild);
    }
}

std::pair<std::vector<Triangle*>, std::vector<Triangle*>> LoadMesh::partitionTriangles(std::vector<Triangle*> triangles, int axis) {
    std::vector<Triangle*> left;
    std::vector<Triangle*> right;

    // Calculate median centroid along the selected axis
    std::vector<float> centroids;
    for (const auto& triangle : triangles) {
        centroids.push_back(triangle->getCentroid()[axis]);
    }
    std::nth_element(centroids.begin(), centroids.begin() + centroids.size() / 2, centroids.end());
    float median = centroids[centroids.size() / 2];

    // Partition triangles based on median
    for (const auto& triangle : triangles) {
        if (triangle->getCentroid()[axis] < median) {
            left.push_back(triangle);
        } else {
            right.push_back(triangle);
        }
    }

    // Handle cases where all triangles fall on one side
    if (left.empty() || right.empty()) {
        size_t mid = triangles.size() / 2;
        left.assign(triangles.begin(), triangles.begin() + mid);
        right.assign(triangles.begin() + mid, triangles.end());
    }

    return {left, right};
}

std::pair<Vector3, Vector3> LoadMesh::triangleBounds(std::vector<Triangle*> triangles) {
    if (triangles.empty()) {
        // Handle empty triangle list
        return {Vector3(0, 0, 0), Vector3(0, 0, 0)};
    }

    Vector3 minBoundsTri(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
    Vector3 maxBoundsTri(-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity());

    for (const Triangle* triangle : triangles) {
        Vector3 vertices[] = {triangle->v0, triangle->v1, triangle->v2};

        for (const auto& vertex : vertices) {
            if (vertex.getX() < minBoundsTri.getX()) minBoundsTri.setX(vertex.getX());
            if (vertex.getY() < minBoundsTri.getY()) minBoundsTri.setY(vertex.getY());
            if (vertex.getZ() < minBoundsTri.getZ()) minBoundsTri.setZ(vertex.getZ());

            if (vertex.getX() > maxBoundsTri.getX()) maxBoundsTri.setX(vertex.getX());
            if (vertex.getY() > maxBoundsTri.getY()) maxBoundsTri.setY(vertex.getY());
            if (vertex.getZ() > maxBoundsTri.getZ()) maxBoundsTri.setZ(vertex.getZ());
        }
    }
    return {minBoundsTri, maxBoundsTri};
}

void LoadMesh::cleanBVH(BVHNode *node) {
    if (!node->getLeaf()) {
        node->getTriangles().clear();
    }
    if (node->getNodeLeft() != nullptr) {
        cleanBVH(node->getNodeLeft());
    }
    if (node->getNodeRight() != nullptr) {
        cleanBVH(node->getNodeRight());
    }
}

void LoadMesh::numChildren(BVHNode* node, int &num) {
    if (node->getLeaf()) {
        std::cout<<"Triangles size: "<<node->getTriangles().size()<<std::endl;
        std::cout<<"Object Bounds"<<std::endl;
        node->getBounds().first.print();
        node->getBounds().second.print();
        std::cout<<"Vertex Positions"<<std::endl;
        for (Triangle* triangle : node->getTriangles()) {
            triangle->v0.print();
            triangle->v1.print();
            triangle->v2.print();
        }
        std::cout<<"------------"<<std::endl;
    }

    if (node->getNodeLeft() != nullptr) {
        //num++;
        numChildren(node->getNodeLeft(), num);
    }
    if (node->getNodeRight() != nullptr) {
        //num++;
        numChildren(node->getNodeRight(), num);
    }
}
