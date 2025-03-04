#include "InputManager.h"

#include "SystemManager.h"
#include "UI.h"

void InputManager::getClickedObject(int x, int y) {
    UI::selectedObject = systemManager->getClickedObject(x, y);
}

void InputManager::setSystemManager(SystemManager* systemManager) {
    this->systemManager = systemManager;
}
