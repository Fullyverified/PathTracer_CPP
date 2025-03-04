#ifndef MATERIALMANAGER_H
#define MATERIALMANAGER_H

#include <string>
#include <unordered_map>

#include "Material.h"
#include "Vector3.h"

class MaterialManager {
public:

    MaterialManager() {

        Material defaultMaterial{Vector3(1, 1, 1), 0.75, 0, 1, 0, 0};
        materials["Default"] = defaultMaterial;

        addDefaultMaterials();
    }

    ~MaterialManager() = default;

    void loadMaterial(std::string& matName) {
        // from json file
    }

    Material& getMaterial(std::string matName) {

        if (materials.find(matName) != materials.end()) {
            return materials[matName];
        }

        std::cout<<"Material Doesn't exit"<<std::endl;
        return materials["Default"];
    }

    void createMaterial(std::string& matName, Material& mat) {
        materials[matName] = Material{mat.colour, mat.roughness, mat.metallic, mat.IOR, mat.transmission, mat.emission};
        /*materials[matName].colour = mat.colour;
        materials[matName].roughness = mat.roughness;
        materials[matName].metallic = mat.metallic;
        materials[matName].IOR = mat.IOR;
        materials[matName].transmission = mat.transmission;
        materials[matName].emission = mat.emission;*/
    }

    void deleteMaterial(std::string& matName) {

        if (matName != "Default") {
            materials.erase(matName);
        } else {
            std::cout<<"Tried to delete default material"<<std::endl;
        }

    }

    void editMaterial(std::string& matName, Material& mat) {
        materials[matName] = Material{mat.colour, mat.roughness, mat.metallic, mat.IOR, mat.transmission, mat.emission};
    }

    void editMaterialColour(std::string& matName, Vector3 colour) {
        materials[matName].colour = colour;
    }

    void editMaterialRoughness(std::string& matName, float roughness) {
        materials[matName].roughness = roughness;
    }

    void editMaterialMetallic(std::string& matName, float metallic) {
        materials[matName].metallic = metallic;
    }

    void editMaterialIOR(std::string& matName, float IOR) {
        materials[matName].IOR = IOR;
    }

    void editMaterialTransmission(std::string& matName, float transmission) {
        materials[matName].transmission = transmission;
    }

    void editMaterialEmission(std::string& matName, float emission) {
        materials[matName].emission = emission;
    }

    void addDefaultMaterials() {
        materials["White"] = Material{Vector3(1, 1, 1), 0.75, 0, 1, 0, 0};
        materials["Red"] = Material{Vector3(1, 0, 0), 0.75, 0, 1, 0, 0};
        materials["Green"] = Material{Vector3(0, 1, 0), 0.75, 0, 1, 0, 0};
        materials["Blue"] = Material{Vector3(0, 0, 1), 0.75, 0, 1, 0, 0};

        materials["Metal"] = Material{Vector3(1, 1, 1), 0, 1, 1, 0, 0};
        materials["Copper"] = Material{Vector3(0.66, 0.5, 0.2), 0, 1, 1, 0, 0};
        materials["Plastic"] = Material{Vector3(1, 1, 1), 0, 0, 1, 0, 0};
        materials["RedPlastic"] = Material{Vector3(1, 0, 0), 0.8, 0, 1, 0, 0};
        materials["GreenPlastic"] = Material{Vector3(0, 1, 0), 0, 0, 1, 0, 0};
        materials["BluePlastic"] = Material{Vector3(0, 0, 1), 0.8, 0, 1, 0, 0};
        materials["OrangePlastic"] = Material{Vector3(0.66, 0.5, 0.2), 0, 0, 1, 0, 0};

        materials["Mirror"] = Material{Vector3(1, 1, 1), 0, 1, 1, 0, 0};
        materials["Glass"] = Material{Vector3(1, 1, 1), 0.0, 0, 1.5, 1, 0};
        materials["Diamond"] = Material{Vector3(1, 1, 1), 0.0, 0, 1, 1, 0};

        materials["Light"] = Material{Vector3(1, 1, 1), 0.75, 0, 1, 0, 15};
        materials["RedGlow"] = Material{Vector3(1, 0, 0), 0.75, 0, 1, 0, 3};
        materials["BlueGlow"] = Material{Vector3(0, 1, 0), 0.75, 0, 1, 0, 3};
        materials["GreenGlow"] = Material{Vector3(0, 0, 1), 0.75, 0, 1, 0, 3};

        materials["SmoothPlastic"] = Material{Vector3(1, 1, 1), 0.05, 0, 1, 0, 0};
        materials["SmoothMetal"] = Material{Vector3(1, 1, 1), 0.05, 1, 1, 0, 0};
        materials["RoughPlastic"] = Material{Vector3(1, 1, 1), 0.8, 0, 1, 0, 0};
        materials["RoughMetal"] = Material{Vector3(1, 1, 1), 0.8, 1, 1, 0, 0};
    }

    std::unordered_map<std::string, Material>& getMaterialMap() {
        return materials;
    }

    bool materialExists(const std::string& matName) {
        if (materials.find(matName) != materials.end()) {
            return true;
        }
        return false;
    }


private:
    std::unordered_map<std::string, Material> materials;
};



#endif //MATERIALMANAGER_H
