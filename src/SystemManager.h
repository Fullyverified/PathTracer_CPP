#ifndef SYSTEMMANAGER_H
#define SYSTEMMANAGER_H

#include "Config.h"
#include "CPUPT.h"
#include "SDL.h"
#include "Window.h"
#include "Renderer.h"
#include "Camera.h"
#include "InputManager.h"
#include "UI.h"

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"
#include "MaterialManager.h"
#include "SceneObjectManager.h"

class SystemManager {
public:
    SystemManager() : running(true), lockInput(true) {
        int width = static_cast<int>(config.resX);
        int height = static_cast<int>(config.resX / (config.aspectX / config.aspectY));

        window = new Window("Path Tracer", width, height);
        renderer = new Renderer(*window, width, height);

        materialManager = new MaterialManager();
        UI::materialManager = materialManager;

        sceneObjectManager = new SceneObjectManager(materialManager);
        UI::sceneObjectManager = sceneObjectManager;

        cpupt = new CPUPT(this, sceneObjectManager->getSceneObjects());

    }

    ~SystemManager() {
        ImGui_ImplSDLRenderer2_Shutdown();
        ImGui_ImplSDL2_Shutdown();
        ImGui::DestroyContext();

        cpupt->deleteObjects();

        delete window;
        delete renderer;
        delete materialManager;
    }

    void initialize() {
        // --- Initialize ImGui ---
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO &io = ImGui::GetIO();
        ImGui::StyleColorsDark();
        ImGui_ImplSDL2_InitForSDLRenderer(window->getSDLWindow(), renderer->getSDLRenderer());
        ImGui_ImplSDLRenderer2_Init(renderer->getSDLRenderer());

        cpupt->initialiseObjects();

        inputManager = new InputManager(sceneObjectManager->getCamera(), window);
        inputManager->setSystemManager(this);

        resX = config.resX;
        resY = config.resY;
        int res = resX * resY;
        RGBBuffer = new uint8_t[res * 3];
    }

    void update(float deltaTime) {
        // process physics
    }

    void render() {
        // path trace scene
        std::cout<<"Launching render thread"<<std::endl;
        cpupt->launchRenderThread();
    }

    void renderCleanUp() {
        cpupt->joinRenderThread();
    }

    void changeResolution() {
        if (UI::resizeBuffer) {
            resX = config.resX;
            resY = config.resY;
            int res = resX * resY;

            RGBBuffer = new uint8_t[res * 3];

            renderer->initializeTexture(resX, resY);
            SDL_SetWindowSize(window->getSDLWindow(), resX, resY);

            UI::resizeBuffer = false;
        }
    }

    void presentScreen() {
        changeResolution();

        renderer->clearScreen();

        // Push RGB Buffer
        renderer->pushRGBBuffer(RGBBuffer, resX);

        if (!inputManager->getHideUI()) {
            // Start ImGui frame
            ImGui_ImplSDL2_NewFrame();
            ImGui_ImplSDLRenderer2_NewFrame();
            ImGui::NewFrame();

            // Render UI
            UI::renderSettings();
            UI::materialEditor();
            UI::sceneEditor();

            // Render ImGui
            ImGui::Render();
            ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer->getSDLRenderer());
        }

        // present screen - ImGui + RGB Buffer
        renderer->presentScreen();

        running = running == false ? false : inputManager->getIsRunning(); // silly little work around to prevent running being overrided
    }

    InputManager *getInputManager() {
        return inputManager;
    }

    void updateRGBBuffer(uint8_t *newRGBBuffer) {
        RGBBuffer = newRGBBuffer;
    }

    bool getIsRunning() {
        return running;
    }

    void setIsRunning(bool running) {
        this->running = running;
    }

    MaterialManager *getMaterialManager() {
        return materialManager;
    }

    SceneObjectManager *getSceneObjectManager() {
        return sceneObjectManager;
    }

    SceneObject* getClickedObject(int x, int y) {
        return cpupt->getClickedObject(x, y);
    }

private:
    CPUPT *cpupt; // CPU path tracer
    Window *window; // SDL_Wndow
    Renderer *renderer; // SDL_Renderer
    InputManager *inputManager;
    MaterialManager *materialManager;
    SceneObjectManager *sceneObjectManager;

    uint8_t *RGBBuffer;
    int resX, resY;

    bool running, lockInput;
};

#endif //SYSTEMMANAGER_H
