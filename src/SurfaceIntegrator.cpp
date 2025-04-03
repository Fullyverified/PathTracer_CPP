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

Vector3 SurfaceIntegrator::fresnelSchlickMetallic(float cosTheta, Vector3 F0) const {
    return F0 + (Vector3(1.0f) - F0) * std::pow(1.0f - cosTheta, 5.0f);
}

float SurfaceIntegrator::fresnelSchlickIOR(float cosTheta, float ior) const {
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

Vector3 SurfaceIntegrator::computeRefractionBRDF(Vector3 col, float F, float n1, float n2, float cosThetaI) const {
    float eta = n1 / n2; // Incident to transmitted index ratio
    return (1 - F) * col * (eta * eta) / cosThetaI;
}

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

BRDF_PDF SurfaceIntegrator::throughputBRDF(Vector3 &wo, Vector3 &wi, Material *mat, Vector3 &n, float R0, SampleType type, bool internal) const {
    // Compute common terms
    float p_specular = mat->metallic;
    float p_transmission = mat->transmission * (1 - mat->metallic);
    float p_diffuse = 1 - (p_specular + p_transmission);

    Vector3 h = Vector3::halfVector(wo, wi);
    float D = distrubtionGGX(n, h, mat->roughness);
    float cosTheta_wi = std::abs(n.dot(wi));
    float cosTheta_wo = std::abs(Vector3::dot(wo, n));
    float cosTheta_H = std::abs(Vector3::dot(wo, h));

    // compute BRDF depending on sample type
    if (type == metallic || type == specularFresnel) {
        float PDF = microfacetPDF(n, wo, wi, D, h);
        //PDF *= type == metallic ? p_specular : R0 * p_transmission;
        if (PDF < 1e-6f) return {};
        Vector3 F = type == metallic ? fresnelSchlickMetallic(cosTheta_wo, mat->colour) : fresnelSchlickIOR(cosTheta_wo, mat->IOR);
        float G = geometrySmithGGX(n, wo, wi, mat->roughness);
        Vector3 brdf_specular = computeMicrofacetBRDF(D, F, G, n, wo, wi);
        return {(brdf_specular * cosTheta_wi) / PDF, brdf_specular, PDF};

    }
    if (type == refrecation) {
        float PDF = refractionPDF();
        if (PDF < 1e-6f) return {};
        float N1, N2;
        if (!internal) {
            N1 = 1.0003;
            N2 = mat->IOR;
        } else {
            N1 = mat->IOR;
            N2 = 1.0003;
            n *= -1.0f;
        }
        cosTheta_wi = std::abs(n.dot(wi));
        cosTheta_wo = std::abs(Vector3::dot(wo, n));
        float F = fresnelSchlickIOR(cosTheta_wi, mat->IOR);
        Vector3 brdf_refraction = computeRefractionBRDF(mat->colour, F, N1, N2, cosTheta_wi);
        return {brdf_refraction * cosTheta_wo, brdf_refraction, PDF};
    }

    if (type == diffuse) {
        float PDF = diffusePDF(cosTheta_wi);
        if (PDF < 1e-6f) return {0};
        Vector3 brdf_diffuse = diffuseBRDF(mat->colour);
        return {(brdf_diffuse * cosTheta_wi) / PDF, brdf_diffuse, PDF};
    }

    return {1.0f};

}

BRDF_PDF SurfaceIntegrator::throughputNEE(Vector3 wo, Vector3 wi, const Material *mat, Vector3 n) const {
    Vector3 h = Vector3::halfVector(wo, wi);
    float cosTheta_wo = std::abs(Vector3::dot(wo, n));
    float cosTheta_wi = std::abs(Vector3::dot(wi, n));
    float cosTheta_H = std::abs(Vector3::dot(wo, h));

    // Compute common terms
    float p_specular = mat->metallic;
    float p_transmission = mat->transmission * (1 - mat->metallic);
    float p_diffuse = 1 - (p_specular + p_transmission);

    float D = distrubtionGGX(n, h, mat->roughness);
    float G = geometrySmithGGX(n, wo, wi, mat->roughness);
    float R0 = fresnelSchlickIOR(cosTheta_H, mat->IOR);

    // PDFs
    float pdf_specular = microfacetPDF(n, wo, wi, D, h);
    float pdf_refraction = refractionPDF();
    float pdf_diffuse = diffusePDF(cosTheta_wi);
    float effective_pdf_specular = pdf_specular * p_specular; // metallic branch

    float effective_pdf_refraction = (R0 * pdf_specular + (1 - R0) * pdf_refraction) * p_transmission; // refraction branch

    float effective_pdf_diffuse = pdf_diffuse * p_diffuse; // diffuse branch
    float effective_pdf = effective_pdf_specular + effective_pdf_refraction + effective_pdf_diffuse;

    // BRDFs
    // Metallic specular
    Vector3 F_metallic = fresnelSchlickMetallic(cosTheta_H, mat->colour);
    Vector3 brdf_metallic;
    if (cosTheta_wo > 0 && cosTheta_wi > 0) {
        // verify directions are in same hemisphere
        brdf_metallic = computeMicrofacetBRDF(D, F_metallic, G, n, wo, wi);
    } else {
        brdf_metallic = Vector3(0, 0, 0);
    }
    // Dielectric specular
    Vector3 F_dielectric(R0);
    Vector3 brdf_specular_diffuse = computeMicrofacetBRDF(D, F_dielectric, G, n, wo, wi);

    // Diffuse
    Vector3 brdf_diffuse = diffuseBRDF(mat->colour);

    // Refraction (no refraction for direct sampling)
    Vector3 brdf_refraction(0, 0, 0);

    // Combine based on material properties
    Vector3 combined_brdf = p_specular * brdf_metallic + p_transmission * ((R0) * brdf_specular_diffuse + ((1 - R0) * brdf_refraction)) + p_diffuse * brdf_diffuse;

    //return combined_brdf;
    // External, include all PDFs
    return {0, combined_brdf, effective_pdf};
}

float SurfaceIntegrator::evaluatePDF(Vector3 wo, Vector3 wi, const Material *mat, Vector3 n) const {
    Vector3 h = Vector3::halfVector(wo, wi);
    float D = distrubtionGGX(n, h, mat->roughness);
    float cosTheta_wi = std::abs(n.dot(wi));
    float R0 = fresnelSchlickIOR(cosTheta_wi, mat->IOR);

    // compute pdf
    float pdf_specular = microfacetPDF(n, wo, wi, D, h);
    float pdf_refraction = refractionPDF();
    float pdf_diffuse = diffusePDF(cosTheta_wi);

    float p_specular = mat->metallic;
    float effective_pdf_specular = pdf_specular * p_specular; // metallic branch

    float p_transmission = mat->transmission * (1 - mat->metallic);
    float effective_pdf_refraction = (R0 * pdf_specular + (1 - R0) * pdf_refraction) * p_transmission; // refraction branch

    float p_diffuse = 1 - (p_specular + p_transmission);
    float effective_pdf_diffuse = pdf_diffuse * p_diffuse; // diffuse branch

    float effective_pdf = effective_pdf_specular + effective_pdf_refraction + effective_pdf_diffuse;
    return effective_pdf;
}