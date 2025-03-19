#include "SurfaceIntegrator.h"
#include "CPUPT.h"

float SurfaceIntegrator::distrubtionGGX(const Vector3 &normal, const Vector3 &halfVector, float roughness) const {
    roughness = std::max(roughness, 0.001f);
    float alpha = roughness * roughness;
    float NdotH = std::max(normal.dot(halfVector), 0.0f);
    float denom = (NdotH * NdotH * (alpha - 1.0f) + 1.0f);

    return alpha / (std::numbers::pi * denom * denom);
}

float SurfaceIntegrator::geometrySchlickGGX(float NdotV, float roughness) const {
    roughness = std::max(roughness, 0.00001f);
    float alpha = roughness * roughness;
    float k = (alpha + 1.0f) * (alpha + 1.0f) / 8.0f;
    return NdotV / (NdotV * (1.0f - k) + k);
}

float SurfaceIntegrator::geometrySmithGGX(const Vector3 &normal, const Vector3 &viewDir, const Vector3 &lightDir, float roughness) const {
    roughness = std::max(roughness, 0.00001f);
    float alpha = roughness * roughness;
    float NdotV = std::max(normal.dot(viewDir), 0.0f);
    float NdotL = std::max(normal.dot(lightDir), 0.0f);
    float ggxV = geometrySchlickGGX(NdotV, alpha);
    float ggxL = geometrySchlickGGX(NdotL, alpha);
    return ggxV * ggxL;
}

Vector3 SurfaceIntegrator::fresnelSchlickSpecular(float cosTheta, Vector3 F0) const {
    return F0 + (Vector3(1.0f) - F0) * std::pow(1.0f - cosTheta, 5.0f);
}

float SurfaceIntegrator::fresnelSchlickRefraction(float cosTheta, float ior) const {
    float R0 = (ior - 1.0003f) / (ior + 1.0003f);
    R0 = R0 * R0;
    return R0 + (1.0f - R0) * std::pow(1.0f - cosTheta, 5.0f);
}

Vector3 SurfaceIntegrator::computeMicrofacetBRDF(float D, Vector3 &F0, float G, Vector3 &n, Vector3 &wo, Vector3 &wi) const {
    // specular
    Vector3 numerator = D * F0 * G;
    float denominator = 4.0f * fabs(n.dot(wo)) * fabs(n.dot(wi));
    return numerator / denominator;
}

float SurfaceIntegrator::microfacetPDF(Vector3 &n, Vector3 &wo, Vector3 &wi, float D, Vector3 &h) const {
    // specular
    float nDotH = fabs(n.dot(h));
    float woDotH = fabs(wo.dot(h));

    /*if (nDotH < 1e-8f || woDotH < 1e-8f) {
        std::cout<<"nDotH < 1e-8f"<<std::endl;
        std::cout<<"nDotH < 1e-8f"<<std::endl;
    }*/

    return (D * nDotH) / (4.0f * woDotH);
}

Vector3 SurfaceIntegrator::computeRefractionBRDF(Vector3 col, float F, float n1, float n2) const {
    // refraction
    return (1 - F) * ((n1 / n2) * (n1 / n2)) * col;
};

float SurfaceIntegrator::refractionPDF() const {
    // refraction
    // For a delta lobe, the PDF = 1
    // because sampling that single direction has probability 1 in that branch.
    return 1;
}

Vector3 SurfaceIntegrator::diffuseBRDF(Vector3 col) const {
    // diffuse
    return col / std::numbers::pi;
}

float SurfaceIntegrator::diffusePDF(float cosTheta) const {
    // diffuse
    return cosTheta / std::numbers::pi;
}

Vector3 SurfaceIntegrator::computeThroughput(Vector3 &wo, Vector3 &wi, Material *mat, Vector3 &n, float R0, SampleType type, bool internal) const {
    Vector3 brdf;

    // compute common terms
    Vector3 h = Vector3::normalOfHalfAngle(wo, wi);
    float D = distrubtionGGX(n, h, mat->roughness);
    float cosTheta_wi = std::abs(n.dot(wi));
    float cosTheta_wo = std::abs(Vector3::dot(wo, n));

    // compute pdf
    float pdf_specular = microfacetPDF(n, wo, wi, D, h);
    float pdf_refraction = refractionPDF();
    float pdf_diffuse = diffusePDF(cosTheta_wi);

    // compute BRDF depending on sample type
    if (type == metallic || type == specularFresnel) {
        Vector3 F = type == metallic ? fresnelSchlickSpecular(cosTheta_wi, mat->colour) : fresnelSchlickRefraction(cosTheta_wi, mat->IOR);
        float G = geometrySmithGGX(n, wo, wi, mat->roughness);
        Vector3 brdf_specular = computeMicrofacetBRDF(D, F, G, n, wo, wi);
        brdf = brdf_specular;
    } else if (type == refreaction) {
        float cosTheta = std::abs(Vector3::dot(wo, n));
        float F = fresnelSchlickRefraction(cosTheta, mat->IOR);
        float N1, N2;
        if (!internal) {
            N1 = 1.0003;
            N2 = mat->IOR;
        } else {
            N1 = mat->IOR;
            N2 = 1.0003;
        }
        brdf = computeRefractionBRDF(mat->colour, F, N1, N2);
    } else if (type == diffuse) {
        Vector3 brdf_diffuse = diffuseBRDF(mat->colour);
        brdf = brdf_diffuse;
    } else {
        return {1.0f};
    }

    if (internal) {
        // Internal - only refraction, therefore only refraction PDF which is 1
        return brdf * cosTheta_wi;
    }
    // External, include all PDFs

    float p_specular = mat->metallic;
    float effective_pdf_specular = pdf_specular * p_specular; // metallic branch

    float p_transmission = mat->transmission * (1 - mat->metallic);
    float effective_pdf_refraction = (R0 * pdf_specular + (1 - R0) * pdf_refraction) * p_transmission; // refraction branch

    float p_diffuse = 1 - (p_specular + p_transmission);
    float effective_pdf_diffuse = (R0 * pdf_specular + (1 - R0) * pdf_diffuse) * p_diffuse; // diffuse branch

    float effective_pdf = effective_pdf_specular + effective_pdf_refraction + effective_pdf_diffuse;

    return (brdf * cosTheta_wi) / effective_pdf;
}

Vector3 SurfaceIntegrator::evaluateBRDF(Vector3 wo, Vector3 wi, const Material *mat, Vector3 n) const {

    float cosTheta_wo = std::abs(Vector3::dot(wo, n));
    float cosTheta_wi = std::abs(Vector3::dot(wi, n));

    // Compute common terms
    float p_specular = mat->metallic;
    float p_transmission = mat->transmission * (1 - mat->metallic);
    float p_diffuse = 1 - (p_specular + p_transmission);

    Vector3 h = Vector3::normalOfHalfAngle(wo, wi);
    float D = distrubtionGGX(n, h, mat->roughness);
    float G = geometrySmithGGX(n, wo, wi, mat->roughness);

    // Metallic specular
    Vector3 F_metallic = fresnelSchlickSpecular(cosTheta_wi, mat->colour);
    Vector3 brdf_metallic;
    if (cosTheta_wo > 0 && cosTheta_wi > 0) { // verify directions are in same hemisphere
        brdf_metallic = computeMicrofacetBRDF(D, F_metallic, G, n, wo, wi);
    } else {
        brdf_metallic = Vector3(0, 0, 0);
    }
    // Dielectric specular
    float R0 = fresnelSchlickRefraction(cosTheta_wi, mat->IOR);
    Vector3 F_dielectric(R0, R0, R0);
    Vector3 brdf_specular_diffuse = computeMicrofacetBRDF(D, F_dielectric, G, n, wo, wi);

    // Diffuse
    Vector3 brdf_diffuse = diffuseBRDF(mat->colour);

    // Refraction (no refraction for direct sampling)
    Vector3 brdf_refraction(0, 0, 0);

    // Combine based on material properties
    Vector3 combined_brdf = p_specular * brdf_metallic + p_transmission * ((R0) * brdf_specular_diffuse + ((1 - R0) * brdf_refraction)) + p_diffuse * ((R0) * brdf_specular_diffuse + ((1 - R0) * brdf_diffuse));
    return combined_brdf;
}

float SurfaceIntegrator::evaluatePDF(Vector3 wo, Vector3 wi, const Material *mat, Vector3 n) const {
    Vector3 h = Vector3::normalOfHalfAngle(wo, wi);
    float D = distrubtionGGX(n, h, mat->roughness);
    float cosTheta_wi = std::abs(n.dot(wi));
    float R0 = fresnelSchlickRefraction(cosTheta_wi, mat->IOR);

    // compute pdf
    float pdf_specular = microfacetPDF(n, wo, wi, D, h);
    float pdf_refraction = refractionPDF();
    float pdf_diffuse = diffusePDF(cosTheta_wi);

    float p_specular = mat->metallic;
    float effective_pdf_specular = pdf_specular * p_specular; // metallic branch

    float p_transmission = mat->transmission * (1 - mat->metallic);
    float effective_pdf_refraction = (R0 * pdf_specular + (1 - R0) * pdf_refraction) * p_transmission; // refraction branch

    float p_diffuse = 1 - (p_specular + p_transmission);
    float effective_pdf_diffuse = (R0 * pdf_specular + (1 - R0) * pdf_diffuse) * p_diffuse; // diffuse branch

    float effective_pdf = effective_pdf_specular + effective_pdf_refraction + effective_pdf_diffuse;
    return effective_pdf;
}

// For ReSTIR
float SurfaceIntegrator::computeApproxPDF(Reservoir& q, Reservoir& candidate) const {
    Vector3 dir = candidate.candidatePosition - q.rayPos;
    float distance = dir.length();
    dir.normalise();

    float cosTheta_q = q.n.dot(dir);
    float cosTheta_x = q.candidateNormal.dot(dir);

    if (cosTheta_q <= 0 || cosTheta_x <= 0) return 0.0f; // early exit

    Vector3 BRDF = evaluateBRDF(q.wo, dir, q.hitMat, q.n);
    Vector3 Le = candidate.lightMat->emission * candidate.lightMat->colour;
    float G = (cosTheta_q * cosTheta_x) / distance;

    float approxPDF = Vector3::luminance(BRDF) * Vector3::luminance(Le) * G;
    return approxPDF;
}