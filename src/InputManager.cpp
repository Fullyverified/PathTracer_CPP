#include "InputManager.h"

#include "SystemManager.h"
#include "UI.h"

void InputManager::getClickedObject(int x, int y) {
    UI::selectedObject = systemManager->getClickedObject(x, y);
    UI::materialKey = UI::selectedObject->getMaterial()->name;
    debugRay(x, y);
}

void InputManager::debugRay(int x, int y) {
    systemManager->debugRay(x, y);
}

void InputManager::setSystemManager(SystemManager* systemManager) {
    this->systemManager = systemManager;
}
