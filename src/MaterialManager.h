#ifndef MATERIALMANAGER_H
#define MATERIALMANAGER_H

#include <string>
#include <unordered_map>

#include "Material.h"
#include "Vector3.h"

class MaterialManager {
public:

    MaterialManager() {
        addDefaultMaterials();
        refreshMaterialNames();
    }

    ~MaterialManager() = default;

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
        materials["Diamond"] = new Material{"Diamond", Vector3(1, 1, 1), 0.0, 0, 1, 1, 0};

        materials["Light"] = new Material{"Light", Vector3(1, 1, 1), 0.75, 0, 1, 0, 15};
        materials["RedGlow"] = new Material{"RedGlow", Vector3(1, 0, 0), 0.75, 0, 1, 0, 3};
        materials["BlueGlow"] = new Material{"BlueGlow", Vector3(0, 1, 0), 0.75, 0, 1, 0, 3};
        materials["GreenGlow"] = new Material{"GreenGlow", Vector3(0, 0, 1), 0.75, 0, 1, 0, 3};

        materials["SmoothPlastic"] = new Material{"SmoothPlastic", Vector3(1, 1, 1), 0.05, 0, 1, 0, 0};
        materials["SmoothMetal"] = new Material{"SmoothMetal", Vector3(1, 1, 1), 0.05, 1, 1, 0, 0};
        materials["RoughPlastic"] = new Material{"RoughPlastic", Vector3(1, 1, 1), 0.8, 0, 1, 0, 0};
        materials["RoughMetal"] = new Material{"RoughMetal", Vector3(1, 1, 1), 0.8, 1, 1, 0, 0};
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

private:
    std::unordered_map<std::string, Material*> materials;
    std::vector<const char*> materialNames;
};



#endif //MATERIALMANAGER_H
