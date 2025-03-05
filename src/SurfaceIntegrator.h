#ifndef SURFACEINTEGRATOR_H
#define SURFACEINTEGRATOR_H

#include <numbers>

#include "Vector3.h"
#include "Material.h"

// To aid PDF and BRDF calculation
enum SampleType {
    metallic,
    specularFresnel,    // specular caused by IOR
    refreaction,
    diffuse
};

class SurfaceIntegrator {
public:
    SurfaceIntegrator() {}
    ~SurfaceIntegrator() {}

    float distrubtionGGX(const Vector3 &normal, const Vector3 &halfVector, float roughness) const; // Microfacet Distribution (D)

    float geometrySchlickGGX(float NdotV, float roughness) const; // Geometrey Term (G)

    float geometrySmithGGX(const Vector3 &normal, const Vector3 &viewDir, const Vector3 &lightDir, float roughness) const;

    Vector3 fresnelSchlickSpecular(float cosTheta, Vector3 F0) const;

    float fresnelSchlickRefraction(float cosTheta, float IOR) const;

    Vector3 computeMicrofacetBRDF(float D, Vector3& F0, float G, Vector3& n, Vector3& wo, Vector3& wi) const; // Specular

    float microfacetPDF(Vector3 &n, Vector3& wo, Vector3 &wi, float D, Vector3& h) const; // Specular

    Vector3 computeRefractionBRDF(Vector3 col, float F, float n1, float n2) const; // Refraction

    float refractionPDF() const; // Refraction

    Vector3 diffuseBRDF(Vector3 col) const; // Difuse

    float diffusePDF(float cosTheta) const; // Difuse

    Vector3 computeThroughput(Vector3& wo, Vector3& wi, Material* mat, Vector3& n, float R0, SampleType type, bool internal) const; // Final throughput

    // Vector functions
    float dot(Vector3& first, Vector3& second) const {
        float dot = first.x * second.x + first.y * second.y + first.z * second.z;
        return dot;
    }

    Vector3 mix(const Vector3 &a, const Vector3 &b, float factor) const {
        return a * (1.0f - factor) + b * factor;
    }

    Vector3 normalOfHalfAngle(const Vector3 &wo, const Vector3 &wi) const {
        Vector3 h = wo + wi;
        h.normalise();
        return h;
    }

    Vector3 reflect(Vector3& dir, Vector3& normal) const {
        float dotP = dot(dir, normal);
        return dir - normal * dotP * 2;
    }

private:
};



#endif //SURFACEINTEGRATOR_H
