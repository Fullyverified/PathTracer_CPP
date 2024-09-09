#include "Mesh.h"
#include "Vector3.h"
#include "Ray.h"

Mesh::Mesh(Vector3 pos, Vector3 dir, float R, float G, float B, float RL, float GL, float BL, float roughness, float refrac, float transp) :
pos(pos), dir(dir), R(R), G(G), B(B), RL(RL), GL(GL), BL(BL), roughness(roughness), refrac(refrac), transp(transp) {
    objID = ++objectCounter;

    Vector3 v1(0, 0, 1);
    Vector3 v2(0, 1, 0);
    Vector3 v3(0, 0, -1);

}

bool Mesh::objectCulling(Ray &ray) const {
    return false;
}

bool Mesh::intersectionCheck(Ray &ray) const {
    return false;
}

void Mesh::getNormal(Ray &ray) const {

}

std::pair<Vector3, Vector3> Mesh::getBounds() const {
    Vector3 min(0 ,0 ,0);
    Vector3 max(0 ,0 ,0);
    return {min, max};
}

std::pair<float, float> Mesh::getIntersectionDistance(Ray &ray) const {
    return {0, 0};
}


Vector3 Mesh::getPos() const {
    return pos;
}

Vector3 Mesh::getCol() const {
    return colour;
}

Vector3 Mesh::getLum() const {
    return luminance;
}

float Mesh::getRough() const {
    return roughness;
}

float Mesh::getRefrac() const {
    return refrac;
}

float Mesh::getTransp() const {
    return transp;
}

int Mesh::getObjID() const {
    return objID;
}

void Mesh::printType() const {
    std::cout<<"Type: Mesh"<<std::endl;
}