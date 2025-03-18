#include "UI.h"

#include <thread>
#include <tiny_obj_loader.h>

#include "ImGui.h"
#include "Config.h"
#include "MaterialManager.h"
#include "Material.h"

#include "SceneObjectManager.h"

int UI::RaysPerSecond = 0;
float UI::pathTracingTime = 0;
float UI::denoisingTime = 0;
float UI::toneMappingTime = 0;
float UI::frameTime = 0;
int UI::accumulatedRays = 0;
bool UI::ReSTIR = config.ReSTIR;
bool UI::unbiased = config.unbiased;
int UI::candidateSamples = config.candidateSamples;
int UI::spatialSamplesK = config.spatialSamplesK;
int UI::temporalSampling = 0;
bool UI::ReSTIRGI = config.ReSTIRGI;
int UI::numRays = config.raysPerPixel;
int UI::minBounces = config.minBounces;
int UI::maxBounces = config.maxBounces;
bool UI::accumulateRays = true;
bool UI::sky = config.sky;
bool UI::denoise = config.denoise;
int UI::denoiseIterations = config.denoiseIterations;

int UI::numThreads = std::thread::hardware_concurrency();

int UI::upscale = config.upScale;

float UI::fOV = config.fOV;
float UI::exposure = config.exposure;
bool UI::depthOfField = config.DepthOfField;
float UI::apetureRadius = config.apertureRadius;
float UI::focalDistance = config.focalDistance;

int UI::resX = config.resX;
int UI::resY = config.resY;

bool UI::camUpdate = false;
bool UI::sceneUpdate = false;
bool UI::upscalingUpdate = false;
bool UI::resUpdate = false;
bool UI::resizeBuffer = false;

Vector3 UI::colour = Vector3(1, 1, 1);
float UI::roughness = 0;
float UI::metallic = 0;
float UI::IOR = 0;
float UI::transmission = 0;
float UI::emission = 0;

MaterialManager *UI::materialManager = nullptr;
std::string UI::materialKey = "Default";
std::string UI::newMatName = "";

SceneObject *UI::selectedObject = nullptr;

bool UI::isWindowHovered = false;

void UI::renderSettings() {
    if (ImGui::GetIO().WantCaptureMouse) {
        isWindowHovered = true;
    } else {
        isWindowHovered = false;
    }

    ImGui::SetNextWindowSize(ImVec2(350, 400), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(0.5f);

    ImGui::Begin("Render Settings", nullptr, ImGuiWindowFlags_None);

    ImGui::Text("Rays /s: %d", RaysPerSecond);

    // Frame time
    std::string frameTimeStr = std::to_string(static_cast<int>(frameTime)) + " ms";
    ImGui::Text("Frame Time: %s", frameTimeStr.c_str());

    // Path tracing time
    std::string pathTraceTimeStr = std::to_string(static_cast<int>(pathTracingTime)) + " ms";
    ImGui::Text("Path Trace: %s", pathTraceTimeStr.c_str());

    // Denoising time
    /*std::string denoisingTimeStr = std::to_string(static_cast<int>(denoisingTime)) + " ms";
    ImGui::Text("Denoising: %s", denoisingTimeStr);*/

    // Tone mapping time
    std::string toneMappingTimeStr = std::to_string(static_cast<int>(toneMappingTime)) + " ms";
    ImGui::Text("Tone Mapping: %s", toneMappingTimeStr);

    if (ImGui::Checkbox("Accumulate Rays", &accumulateRays)) {
        camUpdate = true;
    }

    ImGui::SameLine();

    ImGui::Text("%d", accumulatedRays);

    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x); // Set width to the available space
    if (ImGui::SliderInt("##Rays per frame", &numRays, 1, 100, "Rays per frame %i")) {
        camUpdate = true;
        config.raysPerPixel = numRays;
    }

    ImGui::Separator();

    ImGui::Text("Multiple Importance Sampling (MIS)");

    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x); // Set width to the available space
    if (ImGui::SliderInt("##Min ray bounces", &minBounces, 0, 5, "Min Bounces %i")) {
        if (maxBounces < minBounces) {
            maxBounces = minBounces;
            config.maxBounces = maxBounces;
        }
        camUpdate = true;
        config.minBounces = minBounces;
    }

    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x); // Set width to the available space
    if (ImGui::SliderInt("##Max ray bounces", &maxBounces, 0, 100, "Max bounces %i")) {
        if (maxBounces < minBounces) {
            maxBounces = minBounces;
        }
        camUpdate = true;
    }

    ImGui::Separator();

    // ReSTIR
    if (ImGui::Checkbox("ReSTIR", &ReSTIR)) {
        camUpdate = true;
    }

    ImGui::SameLine();

    if (ImGui::Checkbox("Unbiased", &unbiased)) {
        camUpdate = true;
    }

    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
    if (ImGui::SliderInt("##CandidateSamples", &candidateSamples, 1, 64, "Candidate Samples %i")) {
        camUpdate = true;
    }

    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
    if (ImGui::SliderInt("##SpatialSamples", &spatialSamplesK, 0, 5, "Spatial samples %i")) {
        camUpdate = true;
    }

    if (ImGui::SliderInt("##TemporalSamples", &temporalSampling, 0, 5, "Temporal samples %i")) {
        camUpdate = true;
    }

    // ReSTIR GI
    if (ImGui::Checkbox("ReSTIR Global Illumination", &ReSTIRGI)) {
        camUpdate = true;
    }

    ImGui::Separator();

    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x); // Set width to the available space
    if (ImGui::SliderInt("##CPU Threads", &numThreads, 1, std::thread::hardware_concurrency(), "CPU Threads %i")) {
        config.threads = numThreads;
    }

    ImGui::Separator();

    /*// Denoising
    ImGui::Checkbox("Denoising", &denoise);

    ImGui::SameLine();

    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x); // Set width to the available space
    ImGui::SliderInt("##Denoiser Iterations", &denoiseIterations, 0, 5, "Denoise Steps %i");

    ImGui::Separator();*/

    std::string upscaleString = "Upscaling: " + std::to_string(upscale) + "x";
    ImGui::Text(upscaleString.c_str());

    if (ImGui::Button("1x")) {
        upscale = 1;
        camUpdate = true;
        upscalingUpdate = true;
    }

    ImGui::SameLine();

    if (ImGui::Button("2x")) {
        upscale = 2;
        camUpdate = true;
        upscalingUpdate = true;
    }

    ImGui::SameLine();

    if (ImGui::Button("4x")) {
        upscale = 4;
        camUpdate = true;
        upscalingUpdate = true;
    }

    ImGui::SameLine();

    if (ImGui::Button("6x")) {
        upscale = 6;
        camUpdate = true;
        upscalingUpdate = true;
    }

    ImGui::SameLine();

    if (ImGui::Button("10x")) {
        upscale = 10;
        camUpdate = true;
        upscalingUpdate = true;
    }

    ImGui::Separator();

    ImGui::Text("Camera Settings");

    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x); // Set width to the available space
    if (ImGui::SliderFloat("##fOV", &fOV, 1, 180, "fOV %.3f")) {
        camUpdate = true;
    }

    /*if (ImGui::SliderFloat("Exposure", &exposure, 0.1, 10)) {
        camUpdate = true;
    }*/

    if (ImGui::Checkbox("Depth of Field", &depthOfField)) {
        camUpdate = true;
    }

    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x); // Set width to the available space
    if (ImGui::SliderFloat("##Aperture Radius", &apetureRadius, 0, 2, "Aperture Radius %.3f")) {
        config.apertureRadius = apetureRadius;
        camUpdate = true;
    }

    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x); // Set width to the available space
    if (ImGui::SliderFloat("##Focal Distance", &focalDistance, 0, 100, "Focal Distance %.3f")) {
        camUpdate = true;
    }

    ImGui::Separator();

    ImGui::Text("Resolution");

    ImGui::SetNextItemWidth(80); // Set fixed width for the input box
    ImGui::InputInt("##ResX", &resX, 0, 0, ImGuiInputTextFlags_CharsDecimal);

    ImGui::SameLine();
    ImGui::Text("x");
    ImGui::SameLine();

    ImGui::SetNextItemWidth(80);
    ImGui::InputInt("##ResY", &resY, 0, 0, ImGuiInputTextFlags_CharsDecimal);

    ImGui::SameLine();

    if (ImGui::Button("Apply")) {
        config.resX = resX;
        config.resY = resY;
        config.aspectX = resX;
        config.aspectY = resY;
        camUpdate = true;
        resUpdate = true;
    }

    ImGui::Separator();

    ImGui::Text("Hint: Hold right click for mouse look");
    ImGui::Text("Hint: WASD + QE");
    ImGui::Text("Hint: F1 to hide UI");

    ImGui::End();
}

void UI::materialEditor() {
    ImGui::SetNextWindowSize(ImVec2(350, 400), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(0.5f);

    ImGui::Begin("Material Editor", nullptr, ImGuiWindowFlags_None);

    // Find selected material
    std::vector<const char *> materialNames = materialManager->getMaterailNames();
    int currentIndex = -1;
    for (size_t i = 0; i < materialNames.size(); i++) {
        if (materialKey == materialNames[i]) {
            currentIndex = static_cast<int>(i);
            break;
        }
    }

    // Material Creation
    char matNameBuffer[64] = "";
    strncpy(matNameBuffer, newMatName.c_str(), sizeof(matNameBuffer));

    ImGui::InputTextWithHint("##MaterialName", "Material Name", matNameBuffer, sizeof(matNameBuffer));
    newMatName = std::string(matNameBuffer);

    ImGui::SameLine();

    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x); // Set width to the available space
    if (ImGui::Button("Create")) {
        if (!materialManager->materialExists(newMatName)) {
            Material newMaterial{newMatName, Vector3(1, 1, 1), 0, 0, 1, 0, 0};
            materialManager->createMaterial(newMatName, newMaterial);
            materialKey = newMatName;
            sceneObjectManager->updateEmmisiveObjects();
        }
        std::cout << "A Material with the name already exists" << std::endl;
    }

    // Drop down list of materials
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x); // Set width to the available space
    if (ImGui::Combo("##Materials", &currentIndex, materialNames.data(), static_cast<int>(materialNames.size()))) {
        // Ignore the placeholder selection
        materialKey = materialNames[currentIndex];
    }

    // Update local variables to match selected material
    if (materialKey != "") {
        Material *mat = materialManager->getMaterial(materialKey);
        colour = mat->colour;
        roughness = mat->roughness;
        metallic = mat->metallic;
        IOR = mat->IOR;
        transmission = mat->transmission;
        emission = mat->emission;
    }

    // Colour Editing
    float colourArray[3] = {colour.x, colour.y, colour.z};

    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
    if (ImGui::ColorPicker3("##Colour", colourArray, ImGuiColorEditFlags_DisplayRGB | ImGuiColorEditFlags_NoAlpha | ImGuiColorEditFlags_NoSidePreview)) {
        // Update the Vector3 with the new color values
        colour.x = colourArray[0];
        colour.y = colourArray[1];
        colour.z = colourArray[2];
        materialManager->editMaterialColour(materialKey, colour);
        camUpdate = true;
    }

    // General Material Editing

    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
    if (ImGui::SliderFloat("##Roughness", &roughness, 0.0f, 1.0f, "Roughness %.3f")) {
        roughness = roughness > 1 ? 1 : roughness;
        roughness = roughness < 0 ? 0 : roughness;
        materialManager->editMaterialRoughness(materialKey, roughness);
        camUpdate = true;
    }

    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
    if (ImGui::SliderFloat("##Metallic", &metallic, 0.0f, 1.0f, "Metallic %.3f")) {
        metallic = metallic > 1 ? 1 : metallic;
        metallic = metallic < 0 ? 0 : metallic;
        materialManager->editMaterialMetallic(materialKey, metallic);
        camUpdate = true;
    }

    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
    if (ImGui::SliderFloat("##IOR", &IOR, 1.0f, 10.0f, "IOR %.3f")) {
        IOR = IOR < 1 ? 1 : IOR;
        materialManager->editMaterialIOR(materialKey, IOR);
        camUpdate = true;
    }

    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
    if (ImGui::SliderFloat("##Transmission", &transmission, 0.0f, 1.0f, "Transmission %.3f")) {
        transmission = transmission > 1 ? 1 : transmission;
        transmission = transmission < 0 ? 0 : transmission;
        materialManager->editMaterialTransmission(materialKey, transmission);
        camUpdate = true;
    }

    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
    if (ImGui::SliderFloat("##Emission", &emission, 0.0f, 20.0f, "Emission %.3f")) {
        emission = emission < 0 ? 0 : emission;
        materialManager->editMaterialEmission(materialKey, emission);
        camUpdate = true;
    }

    ImGui::End();
}

int UI::primativeSelection = 0;
int UI::meshSelection = 0;

SceneObjectManager *UI::sceneObjectManager = nullptr;

void UI::sceneEditor() {
    ImGui::SetNextWindowSize(ImVec2(350, 400), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(0.5f);

    ImGui::Begin("Scene Editor", nullptr, ImGuiWindowFlags_None);

    if (ImGui::Checkbox("Sky", &sky)) {
        config.sky = sky;
        camUpdate = true;
    }

    // Drop down to select a primative Type
    std::vector<const char *> primativeTypes = sceneObjectManager->getPrimativeTypes();
    ImGui::Combo("##Primative Objects", &primativeSelection, primativeTypes.data(), static_cast<int>(primativeTypes.size()));

    ImGui::SameLine();

    if (ImGui::Button("Add primative")) {
        sceneObjectManager->addPrimative(primativeTypes[primativeSelection]);
        sceneUpdate = true;
        camUpdate = true;
    }

    // Drop down to select a mesh Type
    std::vector<const char *> meshTypes = sceneObjectManager->getMeshTypes();
    ImGui::Combo("##Mesh Objects", &meshSelection, meshTypes.data(), static_cast<int>(meshTypes.size()));

    ImGui::SameLine();

    if (ImGui::Button("Add mesh")) {
        sceneObjectManager->addMesh(meshTypes[meshSelection]);
        sceneUpdate = true;
        camUpdate = true;
    }

    ImGui::Separator();

    if (selectedObject == nullptr) {
        ImGui::Text("No object selected");

        ImGui::End();
        return;
    } else {
        // Find and store all materials
        std::vector<const char *> materialNames = materialManager->getMaterailNames();

        // Find the current material for the selected Object
        int currentIndex = -1;
        for (size_t i = 0; i < materialNames.size(); i++) {
            if (selectedObject->getMaterial()->name == materialNames[i]) {
                currentIndex = static_cast<int>(i);
                break;
            }
        }

        // Display selected Object Type
        std::string objString = "Selected Object: " + selectedObject->getType();
        ImGui::Text(objString.c_str());

        ImGui::SameLine();

        if (ImGui::Button("Delete")) {
            std::cout << "Calling delete method" << std::endl;
            sceneObjectManager->removeSceneObject(selectedObject);
            selectedObject = nullptr;
            sceneUpdate = true;
            camUpdate = true;
        }

        if (selectedObject == nullptr) {
            ImGui::Text("No object selected");

            ImGui::End();
            return;
        }

        ImGui::Text("Select Material");
        // Show the combo box with the placeholder
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x); // Set width to the available space
        if (ImGui::Combo("##Materials", &currentIndex, materialNames.data(), static_cast<int>(materialNames.size()))) {
            selectedObject->setMaterial(materialManager->getMaterial(materialNames[currentIndex]));
            camUpdate = true;
        }

        ImGui::Separator();

        ImGui::Text("Transform");


        ImGui::Text("Translation: ");
        Vector3 position = selectedObject->getPos();

        // X Axis
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x); // Set width to the available space
        if (ImGui::DragFloat("##Position X", &position.x, 0.1f, 0.0f, 0.0f, "X %.3f")) {
            selectedObject->setPos(position);
            sceneUpdate = true;
            camUpdate = true;
        }

        // Y Axis
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x); // Set width to the available space
        if (ImGui::DragFloat("##Position Y", &position.y, 0.1f, 0.0f, 0.0f, "Y %.3f")) {
            selectedObject->setPos(position);
            sceneUpdate = true;
            camUpdate = true;
        }

        // Z Axis
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x); // Set width to the available space
        if (ImGui::DragFloat("##Position Z", &position.z, 0.1f, 0.0f, 0.0f, "Z %.3f")) {
            selectedObject->setPos(position);
            sceneUpdate = true;
            camUpdate = true;
        }

        ImGui::Text("Scale: ");
        Vector3 scale = selectedObject->getScale();

        // X Axis
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x); // Set width to the available space
        if (ImGui::DragFloat("##Scale X", &scale.x, 0.1f, 0.0f, 0.0f, "X %.3f")) {
            selectedObject->setScale(scale);
            sceneUpdate = true;
            camUpdate = true;
        }

        // Y Axis
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x); // Set width to the available space
        if (ImGui::DragFloat("##Scale Y", &scale.y, 0.1f, 0.0f, 0.0f, "Y %.3f")) {
            selectedObject->setScale(scale);
            sceneUpdate = true;
            camUpdate = true;
        }

        // Z Axis
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x); // Set width to the available space
        if (ImGui::DragFloat("##Scale Z", &scale.z, 0.1f, 0.0f, 0.0f, "Z %.3f")) {
            selectedObject->setScale(scale);
            sceneUpdate = true;
            camUpdate = true;
        }

        if (selectedObject->getType() == "Mesh") {
            ImGui::Text("Rotation: ");
            Vector3 direction = selectedObject->getDir();

            // X Axis
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x); // Set width to the available space
            if (ImGui::DragFloat("##Rotation X", &direction.x, 0.1f, 0.0f, 0.0f, "X %.3f")) {
                selectedObject->setDir(direction);
                sceneUpdate = true;
                camUpdate = true;
            }

            // Y Axis
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x); // Set width to the available space
            if (ImGui::DragFloat("##Rotation Y", &direction.y, 0.1f, 0.0f, 0.0f, "Y %.3f")) {
                selectedObject->setDir(direction);
                sceneUpdate = true;
                camUpdate = true;
            }

            // Z Axis
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x); // Set width to the available space
            if (ImGui::DragFloat("##Rotation Z", &direction.z, 0.1f, 0.0f, 0.0f, "Z %.3f")) {
                selectedObject->setDir(direction);
                sceneUpdate = true;
                camUpdate = true;
            }
        }
    }


    ImGui::End();
}
