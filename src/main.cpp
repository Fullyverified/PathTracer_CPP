#include <vector>
#include <Vector3.h>
#include <Camera.h>
#include <Sphere.h>
#include <AABCubeBounds.h>
#include <AABCubeCenter.h>
#include <Render.h>
#include <Config.h>

#include "GLFW/glfw3.h"
#include "glm.hpp"
#include "vulkan/vulkan.h"

int main() {

    /*std::vector<SceneObject*> SceneObjectsList;

    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,3,3),Vector3(14,12,1),1,1,1,0,0,0,0.75,1,0)); // left wall
    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,3,-3),Vector3(14,12,1),1,1,1,0,0,0,0.75,1,0)); // right wall wall

    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,-3,0),Vector3(14,1,7),1,1,1,0,0,0,0.75,1,0)); // floor
    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(10,3,0),Vector3(14,1,7),1,1,1,0,0,0,0.75,1,0)); // roof

    SceneObjectsList.emplace_back(new AABCubeCenter(Vector3(8,0,0),Vector3(1,6,7),1,1,1,0,0,0,0.75,1,0)); // back wall

    SceneObjectsList.emplace_back(new Sphere(Vector3(5,-1.7,-1),0.8,0.8,0.8,1,1,1,0,0,0,0,1,0)); // right sphere on floor
    SceneObjectsList.emplace_back(new Sphere(Vector3(5,-1.7,1),0.8,0.8,0.8,1,1,1,0,0,0,1,1,0)); // left sphere on floor

    SceneObjectsList.emplace_back(new Sphere(Vector3(5,2.5,0),1,0.1,1,1,1,1,40,40,40,0.75,1,0)); // light on ceiling

    Camera *cam = new Camera(config, Vector3(-2, 0, 0), Vector3(1, 0, 0));
    Render render(*cam);
    //render.constructBVHST(SceneObjectsList);
    //render.BVHProfiling();
    render.computePixels(SceneObjectsList, *cam);

    for (SceneObject *obj: SceneObjectsList) {
        delete obj; // delete sceneObjects from heap
    }
    delete cam; // delete cam*/

    glfwInit();

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    GLFWwindow* window = glfwCreateWindow(800, 600, "Vulkan window", nullptr, nullptr);

    uint32_t extensionCount = 0;
    vkEnumerateInstanceExtensionProperties(nullptr, &extensionCount, nullptr);

    std::cout << extensionCount << " extensions supported\n";

    glm::mat4 matrix;
    glm::vec4 vec;
    auto test = matrix * vec;

    while(!glfwWindowShouldClose(window)) {
        glfwPollEvents();
    }

    glfwDestroyWindow(window);

    glfwTerminate();
    return 0;
}
