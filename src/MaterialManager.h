#ifndef MATERIALMANAGER_H
#define MATERIALMANAGER_H

#include <filesystem>
#include <string>
#include <unordered_map>

#include <SDL_image.h>
#include "Material.h"
#include "Vector3.h"
#include "Vector2.h"
#include "Ray.h"
#include "SceneObject.h"

class MaterialManager {
public:

    MaterialManager() {
        loadTextures();
        addDefaultMaterials();
        refreshMaterialNames();
    }

    ~MaterialManager() {
        for (auto& mat : materials) {
            delete mat.second;
        }
        for (auto& texture : albedos) {
            SDL_FreeSurface(texture.second);
        }
    }

    void loadMaterial(std::string& matName) {
        // from json file
    }

    Material* getMaterial(std::string matName) {
        if (materials.find(matName) != materials.end()) {
            refreshMaterialNames();
            return materials[matName];
        }

        return materials["Default"];
    }

    void createMaterial(std::string& matName, Material& mat) {
        materials[matName] = new Material{matName, mat.colour, mat.roughness, mat.metallic, mat.IOR, mat.transmission, mat.emission};
        materialNames.push_back(matName.c_str());
    }

    void deleteMaterial(std::string& matName) {
        if (matName != "Default") {
            materials.erase(matName);
        } else {
            std::cout<<"Tried to delete default material"<<std::endl;
        }
    }

    std::unordered_map<std::string, Material*>& getMaterialMap() {
        return materials;
    }

    std::vector<const char*> getMaterailNames() {
        return materialNames;
    }

    bool materialExists(const std::string& matName) {
        if (materials.find(matName) != materials.end()) {
            return true;
        }
        return false;
    }

    void addDefaultMaterials() {
        materials["Default"] = new Material("Default", Vector3(1, 1, 1), 0.75, 0, 1, 0, 0);

        materials["White"] = new Material{"White", Vector3(1, 1, 1), 0.75, 0, 1, 0, 0};
        materials["Red"] = new Material{"Red", Vector3(1, 0, 0), 0.75, 0, 1, 0, 0};
        materials["Green"] = new Material{"Green", Vector3(0, 1, 0), 0.75, 0, 1, 0, 0};
        materials["Blue"] = new Material{"Blue", Vector3(0, 0, 1), 0.75, 0, 1, 0, 0};

        materials["Metal"] = new Material{"Metal", Vector3(1, 1, 1), 0, 1, 1, 0, 0};
        materials["Copper"] = new Material{"Copper", Vector3(0.66, 0.5, 0.2), 0, 1, 1, 0, 0};
        materials["Plastic"] = new Material{"Plastic", Vector3(1, 1, 1), 0, 0, 1, 0, 0};
        materials["RedPlastic"] = new Material{"RedPlastic", Vector3(1, 0, 0), 0.8, 0, 1, 0, 0};
        materials["GreenPlastic"] = new Material{"GreenPlastic", Vector3(0, 1, 0), 0, 0, 1, 0, 0};
        materials["BluePlastic"] = new Material{"BluePlastic", Vector3(0, 0, 1), 0.8, 0, 1, 0, 0};
        materials["OrangePlastic"] = new Material{"OrangePlastic", Vector3(0.66, 0.5, 0.2), 0, 0, 1, 0, 0};

        materials["Mirror"] = new Material{"Mirror", Vector3(1, 1, 1), 0, 1, 1, 0, 0};
        materials["Glass"] = new Material{"Glass", Vector3(1, 1, 1), 0.0, 0, 1.5, 1, 0};
        materials["Diamond"] = new Material{"Diamond", Vector3(1, 1, 1), 0.0, 0, 2.42, 1, 0};

        materials["Light"] = new Material{"Light", Vector3(1, 1, 1), 0.75, 0, 1, 0, 15};
        materials["RedGlow"] = new Material{"RedGlow", Vector3(1, 0, 0), 0.75, 0, 1, 0, 3};
        materials["GreenGlow"] = new Material{"GreenGlow", Vector3(0, 1, 0), 0.75, 0, 1, 0, 3};
        materials["BlueGlow"] = new Material{"BlueGlow", Vector3(0, 0, 1), 0.75, 0, 1, 0, 3};

        materials["SmoothPlastic"] = new Material{"SmoothPlastic", Vector3(1, 1, 1), 0.05, 0, 1, 0, 0};
        materials["SmoothMetal"] = new Material{"SmoothMetal", Vector3(1, 1, 1), 0.05, 1, 1, 0, 0};
        materials["RoughPlastic"] = new Material{"RoughPlastic", Vector3(1, 1, 1), 0.8, 0, 1, 0, 0};
        materials["RoughMetal"] = new Material{"RoughMetal", Vector3(1, 1, 1), 0.8, 1, 1, 0, 0};

        // Mesh Object Textures
        materials["Companion Cube"] = new Material{"Companion Cube", Vector3(1, 1, 1), 0.3, 0.1, 1, 0, 1, albedos["Companion Cube"], nullptr, nullptr, emissive["Companion Cube"], nullptr};
        materials["Weighted Cube"] = new Material{"Weighted Cube", Vector3(1, 1, 1), 0.3, 0.1, 1, 0, 1, albedos["Weighted Cube"], nullptr, nullptr, emissive["Weighted Cube"], nullptr};
        materials["Button"] = new Material{"Button", Vector3(1, 1, 1), 0.3, 0.1, 1, 0, 0, albedos["Button"]};
        materials["Portal Gun"] = new Material{"Portal Gun", Vector3(1, 1, 1), 0.3, 0.1, 1, 0, 0, albedos["Portal Gun"], roughness["Portal Gun"], metallic["Portal Gun"], nullptr, normal["Portal Gun"]};

    }

    void editMaterial(std::string& matName, Material& mat) {
        materials[matName] = new Material{matName, mat.colour, mat.roughness, mat.metallic, mat.IOR, mat.transmission, mat.emission};
    }

    void editMaterialColour(std::string& matName, Vector3 colour) {
        materials[matName]->colour = colour;
    }

    void editMaterialRoughness(std::string& matName, float roughness) {

        if (materials.find(matName) == materials.end()) {
            std::cout<<"Material does not exit"<<std::endl;
            std::cout<<"Material: "<<matName<<std::endl;
        }

        materials[matName]->roughness = roughness;
    }

    void editMaterialMetallic(std::string& matName, float metallic) {
        materials[matName]->metallic = metallic;
    }

    void editMaterialIOR(std::string& matName, float IOR) {
        materials[matName]->IOR = IOR;
    }

    void editMaterialTransmission(std::string& matName, float transmission) {
        materials[matName]->transmission = transmission;
    }

    void editMaterialEmission(std::string& matName, float emission) {
        materials[matName]->emission = emission;
    }

    void refreshMaterialNames() {
        materialNames.clear();
        for (const auto& pair : materials) {
            materialNames.push_back(pair.first.c_str());
        }
    }

    void loadTextures() {
        albedos["Weighted Cube"] = createTexture("weighted_cube_texture.png");
        albedos["Companion Cube"] = createTexture("companion_cube_texture.png");
        albedos["Button"] = createTexture("button_texture.png");

        emissive["Weighted Cube"] = createTexture("weighted_cube_emission.png");
        emissive["Companion Cube"] = createTexture("companion_cube_emission.png");

        // portal gun
        albedos["Portal Gun"] = createTexture("portal_gun_texture.png");
        roughness["Portal Gun"] = createTexture("portal_gun_rough.png");
        metallic["Portal Gun"] = createTexture("portal_gun_metallic.png");
        emissive["Portal Gun"] = createTexture("portal_gun_emission.png");
        normal["Portal Gun"] = createTexture("portal_gun_normal.png");

    }

    SDL_Surface* createTexture(std::string name) {
        std::filesystem::path textureDir = std::filesystem::current_path() /"assets"/ "textures";
        SDL_Surface* loaded = IMG_Load((textureDir / name).string().c_str());
        if (!loaded) {
            std::cerr << "Failed to load image: " << IMG_GetError() << std::endl;
            return nullptr;
        }

        // Convert to a known pixel format (e.g. SDL_PIXELFORMAT_RGBA32)
        SDL_Surface* converted = SDL_ConvertSurfaceFormat(loaded, SDL_PIXELFORMAT_RGBA32, 0);
        SDL_FreeSurface(loaded); // Free the original loaded surface

        return converted;
    }

    Material* sampleMatFromHitPoint(Ray& ray, const Material* mat, const SceneObject* hitObject) {

        if (!hitObject->isMesh()) return new Material(mat->name, mat->colour, mat->roughness, mat->metallic, mat->IOR, mat->transmission, mat->emission);

        // Compute texture coordinates
        const Vector3 bCoords = ray.getBCoords();

        float u = bCoords.x;
        float v = bCoords.y;
        float w = bCoords.z;

        const Triangle* tri = ray.getTriangle();
        Vector2 uv0 = tri->uv0;
        Vector2 uv1 = tri->uv1;
        Vector2 uv2 = tri->uv2;

        Vector2 uvCoord = uv0 * w + uv1 * u + uv2 * v;

        SDL_Surface* textureMap = mat->textureMap;
        SDL_Surface* roughnessMap = mat->roughnessMap;
        SDL_Surface* metallicMap = mat->metallicMap;
        SDL_Surface* emissionMap = mat->emissionMap;

        // Texture map
        Vector3 texture;
        if (textureMap == nullptr) {
            texture = mat->colour;
        } else {
            int x = static_cast<int>(uvCoord.x * (textureMap->w - 1));
            int y = static_cast<int>((1.0f - uvCoord.y) * (textureMap->h - 1));

            x = std::clamp(x, 0, textureMap->w - 1);
            y = std::clamp(y, 0, textureMap->h - 1);

            Uint32* pixels = (Uint32*)textureMap->pixels;
            Uint32 pixel = pixels[y * textureMap->w + x];

            Uint8 r, g, b, a;
            SDL_GetRGBA(pixel, textureMap->format, &r, &g, &b, &a);
            texture = Vector3(r, g, b);
            texture /= 255.0f;
        }

        // Roughness map
        float roughness;
        if (roughnessMap == nullptr) {
            roughness = mat->roughness;
        } else {
            int x = static_cast<int>(uvCoord.x * (roughnessMap->w - 1));
            int y = static_cast<int>((1.0f - uvCoord.y) * (roughnessMap->h - 1));

            x = std::clamp(x, 0, roughnessMap->w - 1);
            y = std::clamp(y, 0, roughnessMap->h - 1);

            Uint32* pixels = (Uint32*)roughnessMap->pixels;
            Uint32 pixel = pixels[y * roughnessMap->w + x];

            Uint8 r, g, b;
            SDL_GetRGB(pixel, roughnessMap->format, &r, &g, &b);
            roughness = (int)r; // all channels should be equal (greyscale)
            roughness /= 255.0f;
        }

        // Metallic map
        float metallic;
        if (metallicMap == nullptr) {
            metallic = mat->metallic;
        } else {
            int x = static_cast<int>(uvCoord.x * (metallicMap->w - 1));
            int y = static_cast<int>((1.0f - uvCoord.y) * (metallicMap->h - 1));

            x = std::clamp(x, 0, metallicMap->w - 1);
            y = std::clamp(y, 0, metallicMap->h - 1);

            Uint32* pixels = (Uint32*)metallicMap->pixels;
            Uint32 pixel = pixels[y * metallicMap->w + x];

            Uint8 r, g, b;
            SDL_GetRGB(pixel, metallicMap->format, &r, &g, &b);
            metallic = (int)r; // all channels should be equal (greyscale)
            metallic /= 255.0f;
        }

        // Emission map
        float emission;
        if (emissionMap == nullptr) {
            emission = mat->emission;
        } else {
            int x = static_cast<int>(uvCoord.x * (emissionMap->w - 1));
            int y = static_cast<int>((1.0f - uvCoord.y) * (emissionMap->h - 1));

            x = std::clamp(x, 0, emissionMap->w - 1);
            y = std::clamp(y, 0, emissionMap->h - 1);

            Uint32* pixels = (Uint32*)emissionMap->pixels;
            Uint32 pixel = pixels[y * emissionMap->w + x];

            Uint8 r, g, b;
            SDL_GetRGB(pixel, emissionMap->format, &r, &g, &b);
            emission = (int)r; // all channels should be equal (greyscale)
            emission /= 255.0f;
            emission *= mat->emission;
        }

        return new Material(mat->name, texture, roughness, metallic, mat->IOR, mat->transmission, emission, textureMap, roughnessMap, metallicMap, emissionMap);
    }

private:
    std::unordered_map<std::string, Material*> materials;
    std::vector<const char*> materialNames;

    std::unordered_map<std::string, SDL_Surface*> albedos;
    std::unordered_map<std::string, SDL_Surface*> roughness;
    std::unordered_map<std::string, SDL_Surface*> metallic;
    std::unordered_map<std::string, SDL_Surface*> emissive;
    std::unordered_map<std::string, SDL_Surface*> normal;

};



#endif //MATERIALMANAGER_H
