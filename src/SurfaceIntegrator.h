#ifndef SURFACEINTEGRATOR_H
#define SURFACEINTEGRATOR_H

#include <numbers>

#include "Vector3.h"
#include "Material.h"
#include "MaterialManager.h"
#include "Ray.h"

class CPUPT;
struct Reservoir;
struct ReservoirGI;

// To aid PDF and BRDF calculation
enum SampleType {
    metallic,
    specularFresnel,    // specular caused by IOR
    refrecation,
    diffuse
};

struct BRDF_PDF {
    Vector3 throughput = {0.0f};
    Vector3 BRDF = {0.0f};
    float PDF = 0;
};

class SurfaceIntegrator {
public:
    SurfaceIntegrator(MaterialManager* materialManager) : materialManager(materialManager) {}
    ~SurfaceIntegrator() {
        delete materialManager;
    }

    float distrubtionGGX(const Vector3 &normal, const Vector3 &halfVector, float roughness) const; // Microfacet Distribution (D)

    float geometrySchlickGGX(float NdotV, float roughness) const; // Geometrey Term (G)

    float geometrySmithGGX(const Vector3 &normal, const Vector3 &viewDir, const Vector3 &lightDir, float roughness) const;

    Vector3 fresnelSchlickMetallic(float cosTheta, Vector3 F0) const;

    float fresnelSchlickIOR(float cosTheta, float IOR) const;

    Vector3 computeMicrofacetBRDF(float D, Vector3& F0, float G, Vector3& n, Vector3& wo, Vector3& wi) const; // Specular

    float microfacetPDF(Vector3 &n, Vector3& wo, Vector3 &wi, float D, Vector3& h) const; // Specular

    Vector3 computeRefractionBRDF(Vector3 col, float F, float n1, float n2, float cosThetaT) const; // Refraction

    float refractionPDF() const; // Refraction

    Vector3 diffuseBRDF(Vector3 col) const; // Difuse

    float diffusePDF(float cosTheta) const; // Difuse

    BRDF_PDF throughputBSDF(Vector3& wo, Vector3& wi, Material* mat, Vector3& n, float R0, SampleType type, bool internal) const; // Final throughput

    BRDF_PDF throughputNEE(Vector3 wo, Vector3 wi, const Material *mat, Vector3 n) const;

    float evaluatePDF(Vector3 wo, Vector3 wi, const Material *mat, Vector3 n) const;

private:
    MaterialManager* materialManager;
};



#endif //SURFACEINTEGRATOR_H
