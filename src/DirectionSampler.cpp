#include "DirectionSampler.h"

#include <numbers>

thread_local std::mt19937 DirectionSampler::rng(std::random_device{}());

Vector3 DirectionSampler::RefractionDirection(Ray &ray, SceneObject &sceneObject) const {
    Vector3 wi = ray.getDir();
    Vector3 N = ray.getNormal();

    /*if (ray.getDebug()) {
        std::cout<<"----------------------------"<<std::endl;
        std::cout<<"REFRACTION:"<<std::endl;
        std::cout<<"Ray Direction: ";
        wi.print();
        std::cout<<"Normal: ";
        N.print();
    }*/
    float n1 = 1.0003f; // refractive index of air
    float n2 = sceneObject.getMaterial(ray)->IOR;

    if (ray.getInternal()) {
        // inside glass, flip normal, IOR, and dotProduct
        N = N * -1;
        std::swap(n1, n2);

        //if (ray.getDebug()) std::cout<<"Ray Internal, swapping N1, N2, and flipping N"<<std::endl;

    }

    /*if (ray.getDebug()) {
        std::cout<<"N1: "<<n1<<std::endl;
        std::cout<<"N2: "<<n2<<std::endl;
        std::cout<<"Normal: ";
        N.print();
    }*/

    float cosThetaI = -N.dot(wi);

    // cosine of incient angle
    float sinTheta1 = std::sqrt(std::max(0.0f, 1.0f - cosThetaI * cosThetaI));
    float sinTheta2 = (n1 / n2) * sinTheta1;

    /*if (ray.getDebug()) {
        std::cout<<"CosThetaI: "<<cosThetaI<<std::endl;
        std::cout<<"sinTheta1: "<<sinTheta1<<std::endl;
        std::cout<<"sinTheta2: "<<sinTheta2<<std::endl;
    }*/

    if (sinTheta2 >= 1) {
        // total internal reflection - bounce off object / bounce back inside object
        Vector3 reflection = wi.reflect(N);
        reflection.normalise();

        /*if (ray.getDebug()) {
            std::cout<<"SinTheta2 >= 1 - Total Internal Reflection"<<std::endl;
            std::cout<<"Reflection Direction:";
            reflection.print();
        }*/

        return reflection;
    }

    // valid refreaction into next medium
    float cosTheta2 = std::sqrt(std::max(0.0f, 1.0f - sinTheta2 * sinTheta2));
    Vector3 refraction = (wi * (n1 / n2)) + (N * ((n1 / n2) * cosThetaI - cosTheta2));

    refraction.normalise(); // Return normalized refracted direction
    // on first refraction flip internal to true
    // second refraction is always on exit - flips to false
    ray.flipInternal();

    /*if (ray.getDebug()) {
        std::cout<<"SinTheta2 < 1 - Refraction"<<std::endl;
        std::cout<<"CosTheta2: "<<cosTheta2<<std::endl;
        std::cout<<"Refraction Direction: ";
        refraction.print();
    }*/

    return refraction;
}

Vector3 DirectionSampler::SpecularDirection(Ray &ray, const SceneObject &sceneObject, bool flipNormal) const {
    const Material* mat = sceneObject.getMaterial(ray);
    float rough = std::max(mat->roughness, 0.001f);
    float alpha = rough * rough; // for GGX


    Vector3 N = ray.getNormal();
    if (flipNormal) {
        N.flip();
    }

    // Sample half vector H in tanget space
    float r1 = dist(rng);;
    float r2 = dist(rng);

    float phi = 2.0f * std::numbers::pi * r1;
    // "Smith" or "Heitz" form for cosTheta half
    float cosTheta = std::sqrt((1.0f - r2) / (1.0f + (alpha * alpha - 1.0f) * r2));
    float sinTheta = std::sqrt(std::max(0.0f, 1.0f - cosTheta * cosTheta));

    // Half vector in local tangent space
    Vector3 Ht(sinTheta * std::cos(phi),
               sinTheta * std::sin(phi),
               cosTheta);

    // Build an orthonormal basis around N
    Vector3 arbitraryA;
    if (std::abs(N.x) < 0.5f && std::abs(N.z) < 0.5f) {
        arbitraryA = Vector3(1, 0, 0);
    } else {
        arbitraryA = Vector3(0, 1, 0);
    }
    Vector3 T = N.cross(arbitraryA);
    T.normalise();
    Vector3 B = N.cross(T);
    B.normalise();

    // Transform Ht to world space - cross product
    Vector3 H(
        Ht.x * T.x + Ht.y * B.x + Ht.z * N.x,
        Ht.x * T.y + Ht.y * B.y + Ht.z * N.y,
        Ht.x * T.z + Ht.y * B.z + Ht.z * N.z
    );
    H.normalise();

    // Reflect incoming dir around H to get outgoing direction
    Vector3 V = ray.getDir();
    V.normalise();

    float dotVH = V.dot(H);
    if (dotVH < 0.0f) {
        H = H * -1;
        dotVH = -dotVH;
    }
    Vector3 R = V - H * 2.0f * dotVH; // reflection direction in world space

    R.normalise();

    return R;
}

Vector3 DirectionSampler::DiffuseDirection(Ray &ray, const SceneObject &sceneObject, bool flipNormal) const {
    // Diffuse sampling - random direction above surface with cosine-weighted distribution
    // Surface normal
    Vector3 N = ray.getNormal();
    if (flipNormal) {
        N.flip();
    }

    // random numbers for hemisphere sampling
    float alpha = dist(rng);
    float gamma = dist(rng);

    // conver to spherical coordinates

    float theta = std::acos(std::sqrt(alpha)); // polar angle
    float phi = 2.0f * std::numbers::pi * gamma; // azimuth

    // Convert spherical coords to Cartesian in local tangent space
    Vector3 localDir(
        std::sin(theta) * std::cos(phi),
        std::sin(theta) * std::sin(phi),
        std::cos(theta)
    );
    localDir.normalise();

    // Build tangent & bitangent from N
    Vector3 arbitraryA;
    if (std::abs(N.getX()) < 0.5f && std::abs(N.getZ()) < 0.5f) {
        arbitraryA.set(1, 0, 0);
    } else {
        arbitraryA.set(0, 1, 0);
    }
    Vector3 tangentT = N.cross(arbitraryA);
    tangentT.normalise();
    Vector3 bitangent = N.cross(tangentT);
    bitangent.normalise();

    // Transform localDir into world space
    Vector3 worldDir(
        localDir.getX() * tangentT.getX() + localDir.getY() * bitangent.getX() + localDir.getZ() * N.getX(),
        localDir.getX() * tangentT.getY() + localDir.getY() * bitangent.getY() + localDir.getZ() * N.getY(),
        localDir.getX() * tangentT.getZ() + localDir.getY() * bitangent.getZ() + localDir.getZ() * N.getZ()
    );

    worldDir.normalise();

    return worldDir;
}