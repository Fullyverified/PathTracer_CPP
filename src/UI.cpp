#include "UI.h"

#include "ImGui.h"
#include "Config.h"
#include "iostream"

int UI::accumulatedRays = 0;
int UI::numRays = config.raysPerPixel;
int UI::numBounces = config.bounceDepth;
bool UI::accumulateRays = true;
int UI::upscale = config.upScale;

bool UI::depthOfField = config.DepthOfField;
float UI::apetureRadius = config.apertureRadius;
float UI::focalDistance = config.focalDistance;

bool UI::camUpdate = false;
bool UI::sceneUpdate = false;

void UI::render() {

    ImGui::SetNextWindowSize(ImVec2(350, 400), ImGuiCond_FirstUseEver);
    ImGui::Begin("Render Settings", nullptr, ImGuiWindowFlags_NoCollapse);

    ImGui::SetNextWindowBgAlpha(0.7f);
    ImGui::SetNextWindowCollapsed(false, ImGuiCond_Always);


    ImGui::Text("Accumlated Rays: %d", accumulatedRays);

    if (ImGui::Checkbox("Accumulate Rays", &accumulateRays)) {
        camUpdate = true;
    }

    if (ImGui::SliderInt("Number of Rays", &numRays, 1, 100)) {
        camUpdate = true;
        config.raysPerPixel = numRays;
    }

    if (ImGui::SliderInt("Number of Bounces", &numBounces, 0, 100)) {
        camUpdate = true;
        config.bounceDepth = numBounces;
    }

    ImGui::Separator();

    ImGui::Text("Upscaling");

    if (ImGui::Button("1x")) {
        upscale = 1;
        camUpdate = true;
    }

    ImGui::SameLine();

    if (ImGui::Button("2x")) {
        upscale = 2;
        camUpdate = true;
    }

    ImGui::SameLine();

    if (ImGui::Button("4x")) {
        upscale = 4;
        camUpdate = true;
    }

    ImGui::SameLine();

    if (ImGui::Button("6x")) {
        upscale = 6;
        camUpdate = true;
    }
    config.upScale = upscale;

    ImGui::Separator();

    ImGui::Text("Camera Settings");

    if (ImGui::Checkbox("Depth of Field", &depthOfField)) {
        config.DepthOfField = depthOfField;
        camUpdate = true;
    }

    if (ImGui::SliderFloat("Aperture Radius", &apetureRadius, 0, 1000)) {
        camUpdate = true;
        config.apertureRadius = apetureRadius;
    }

    if (ImGui::SliderFloat("Focal Distance", &focalDistance, 0, 1000)) {
        camUpdate = true;
        config.focalDistance = focalDistance;
    }

    ImGui::End();

}