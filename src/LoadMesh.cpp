#include "LoadMesh.h"

#define TINYOBJLOADER_IMPLEMENTATION
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
}

std::pair<Vector3, Vector3> LoadMesh::getBounds() {

    minBounds = std::numeric_limits<float>::infinity();
    maxBounds = -std::numeric_limits<float>::infinity();


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

    std::cout<<"Bounds"<<std::endl;
    minBounds.print();
    maxBounds.print();
    return {minBounds, maxBounds};
}

void createBVH() {



}
