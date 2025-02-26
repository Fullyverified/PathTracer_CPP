#include "UI.h"

#include <thread>
#include <tiny_obj_loader.h>

#include "ImGui.h"
#include "Config.h"
#include "iostream"

int UI::RaysPerSecond = 0;
float UI::pathTracingTime = 0;
float UI::toneMappingTime = 0;
float UI::frameTime = 0;
int UI::accumulatedRays = 0;
int UI::numRays = config.raysPerPixel;
int UI::numBounces = config.bounceDepth;
bool UI::accumulateRays = true;

int UI::numThreads = std::thread::hardware_concurrency();

int UI::upscale = config.upScale;

float UI::fOV = config.fOV;
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

void UI::renderSettings() {

    ImGui::SetNextWindowSize(ImVec2(350, 400), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(0.5f);

    ImGui::Begin("Render Settings", nullptr, ImGuiWindowFlags_None);

    ImGui::Text("Rays /s: %d", RaysPerSecond);

    std::string frameTimeStr = std::to_string(static_cast<int>(frameTime)) + " ms";
    ImGui::Text("Frame Time: %s", frameTimeStr.c_str());

    std::string pathTraceTimeStr = std::to_string(static_cast<int>(pathTracingTime)) + " ms";
    ImGui::Text("Path Trace: %s", pathTraceTimeStr.c_str());

    std::string toneMappingTimeStr = std::to_string(static_cast<int>(toneMappingTime)) + " ms";
    ImGui::Text("Tone Mapping: %s", toneMappingTimeStr);

    if (ImGui::Checkbox("Accumulate Rays", &accumulateRays)) {
        camUpdate = true;
    }

    ImGui::SameLine();

    ImGui::Text("%d", accumulatedRays);

    if (ImGui::SliderInt("Rays per frame", &numRays, 1, 100)) {
        camUpdate = true;
        config.raysPerPixel = numRays;
    }

    if (ImGui::SliderInt("Ray bounces", &numBounces, 0, 100)) {
        camUpdate = true;
        config.bounceDepth = numBounces;
    }

    ImGui::Separator();

    if (ImGui::SliderInt("CPU Threads", &numThreads, 1, std::thread::hardware_concurrency())) {
        config.threads = numThreads;
    }

    ImGui::Separator();

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

    if (ImGui::SliderFloat("fOV", &fOV, 1, 180)) {
        camUpdate = true;
        config.fOV = fOV;
    }

    if (ImGui::Checkbox("Depth of Field", &depthOfField)) {
        config.DepthOfField = depthOfField;
        camUpdate = true;
    }

    if (ImGui::SliderFloat("Aperture Radius", &apetureRadius, 0, 2)) {
        camUpdate = true;
        config.apertureRadius = apetureRadius;
    }

    if (ImGui::SliderFloat("Focal Distance", &focalDistance, 0, 100)) {
        camUpdate = true;
        config.focalDistance = focalDistance;
    }

    ImGui::Separator();

    ImGui::Text("Change Resolution");

    ImGui::SetNextItemWidth(80); // Set fixed width for the input box
    if (ImGui::InputInt("##ResX", &resX, 0, 0, ImGuiInputTextFlags_CharsDecimal)) {

    }

    ImGui::SameLine();ImGui::Text("x");ImGui::SameLine();

    ImGui::SetNextItemWidth(80);
    if (ImGui::InputInt("##ResY", &resY, 0, 0, ImGuiInputTextFlags_CharsDecimal)) {

    }

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

    ImGui::Text("Hint: Press Del to enable mouse look");
    ImGui::Text("Hint: WASD + QE");
    ImGui::Text("Hint: F1 to hide UI");

    ImGui::End();

}

void UI::materialEditor() {

    ImGui::SetNextWindowSize(ImVec2(350, 400), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(0.5f);

    ImGui::Begin("Material Editor", nullptr, ImGuiWindowFlags_None);



    ImGui::Text("Work In Progress");

    ImGui::End();
}

void UI::sceneEditor() {

    ImGui::SetNextWindowSize(ImVec2(350, 400), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(0.5f);

    ImGui::Begin("Scene Editor", nullptr, ImGuiWindowFlags_None);

    ImGui::Text("Work In Progress");



    ImGui::End();
}