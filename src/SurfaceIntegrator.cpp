//
// Created by hazza on 5/03/2025.
//

#include "SurfaceIntegrator.h"

float SurfaceIntegrator::distrubtionGGX(const Vector3 &normal, const Vector3 &halfVector, float roughness) const {
    roughness = std::max(roughness, 0.001f);
    float alpha = roughness * roughness;
    float NdotH = std::max(normal.dot(halfVector), 0.0f);
    float denom = (NdotH * NdotH * (alpha - 1.0f) + 1.0f);

    return alpha / (std::numbers::pi * denom * denom);
}

float SurfaceIntegrator::geometrySchlickGGX(float NdotV, float roughness) const {
    roughness = std::max(roughness, 0.001f);
    float k = (roughness + 1.0f) * (roughness + 1.0f) / 8.0f;
    return NdotV / (NdotV * (1.0f - k) + k);
}

float SurfaceIntegrator::geometrySmithGGX(const Vector3 &normal, const Vector3 &viewDir, const Vector3 &lightDir, float roughness) const {
    roughness = std::max(roughness, 0.001f);
    float NdotV = std::max(normal.dot(viewDir), 0.0f);
    float NdotL = std::max(normal.dot(lightDir), 0.0f);
    float ggxV = geometrySchlickGGX(NdotV, roughness);
    float ggxL = geometrySchlickGGX(NdotL, roughness);
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

    // compute terms
    Vector3 h = Vector3::normalOfHalfAngle(wo, wi);
    float D = distrubtionGGX(n, h, mat->roughness);
    float cosTheta_wi = std::abs(n.dot(wi));

    // compute pdf
    float pdf_specular = microfacetPDF(n, wo, wi, D, h);
    float pdf_refraction = refractionPDF();
    float pdf_diffuse = diffusePDF(cosTheta_wi);

    // compute BRDF depending on sample type
    if (type == metallic || type == specularFresnel) {
        float cosTheta_wo = std::abs(Vector3::dot(wo, n));
        Vector3 F = type == metallic ? fresnelSchlickSpecular(cosTheta_wo, mat->colour) : fresnelSchlickRefraction(cosTheta_wo, mat->IOR);
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
        return 1;
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
    float cosTheta_wo = n.dot(wo);
    float cosTheta_wi = n.dot(wi);

    // Compute common terms
    Vector3 h = Vector3::normalOfHalfAngle(wo, wi);
    float D = distrubtionGGX(n, h, mat->roughness);
    float G = geometrySmithGGX(n, wo, wi, mat->roughness);
    float denom = 4.0f * std::abs(cosTheta_wo) * std::abs(cosTheta_wi);
    if (denom == 0) return Vector3(0);

    float p_specular = mat->metallic;
    float p_transmission = mat->transmission * (1 - mat->metallic);
    float p_diffuse = 1 - (p_specular + p_transmission);

    if (cosTheta_wo * cosTheta_wi > 0) {
        // Reflection
        // Metallic specular
        float cosTheta_h_wi = h.dot(wi);
        Vector3 F_metallic = fresnelSchlickSpecular(cosTheta_h_wi, mat->colour);
        Vector3 brdf_specular_metallic = (D * F_metallic * G) / denom;

        // Dielectric specular
        float F_dielectric = fresnelSchlickRefraction(cosTheta_h_wi, mat->IOR); // Air IOR = 1.0
        Vector3 brdf_specular_dielectric = (D * F_dielectric * G) / denom;

        // Diffuse
        Vector3 brdf_diffuse = mat->colour / std::numbers::pi;

        // Combine based on material properties
        Vector3 brdf = p_specular * brdf_specular_metallic + p_transmission * brdf_specular_dielectric + p_diffuse * brdf_diffuse;
        return brdf;
    }
    // Transmission
    // Only non-metallic, transmissive materials contribute
    float F_dielectric = fresnelSchlickRefraction(std::abs(cosTheta_wo), mat->IOR);
    float n1, n2;
    if (cosTheta_wo > 0) {
        // Exiting material
        n1 = mat->IOR;
        n2 = 1.0f;
    } else {
        // Entering material
        n1 = 1.0f;
        n2 = mat->IOR;
    }
    Vector3 btdf = p_transmission * computeRefractionBRDF(mat->colour, F_dielectric, n1, n2);
    return btdf;

}

float SurfaceIntegrator::evaluatePDF(Vector3 wo, Vector3 wi, const Material *mat, Vector3 n) const {
    float cosTheta_wo = n.dot(wo);
    float cosTheta_wi = n.dot(wi);

    if (cosTheta_wo * cosTheta_wi > 0) {
        // Reflection
        Vector3 h = Vector3::normalOfHalfAngle(wo, wi);
        float D = distrubtionGGX(n, h, mat->roughness);
        float pdf_specular = microfacetPDF(n, wo, wi, D, h);
        float pdf_diffuse = diffusePDF(std::abs(cosTheta_wi));

        float cosTheta_h_wi = h.dot(wi);
        float R0 = fresnelSchlickRefraction(cosTheta_h_wi, mat->IOR);

        float p_specular = mat->metallic;
        float p_transmission = mat->transmission * (1 - mat->metallic);
        float p_diffuse = 1 - p_specular - p_transmission;

        float pdf = p_specular * pdf_specular +
                    p_transmission * R0 * pdf_specular +
                    p_diffuse * (R0 * pdf_specular + (1 - R0) * pdf_diffuse);
        return pdf;
    } else {
        // Transmission
        // For smooth dielectrics, PDF is delta; practically 0 unless wi matches refracted direction
        // In direct lighting with area lights, this is typically handled separately
        return 0.0f;
    }
}
