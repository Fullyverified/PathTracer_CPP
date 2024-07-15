#include <chrono>
#include <iostream>

#include <Vector3.h>

int main() {

    Vector3 rayDir(1,0,0);
    rayDir.normalise();

    Vector3 newPos(1,1,1);

    rayDir.addSIMD(newPos);

    std::cout << "x: " << rayDir.x << ", y: " << rayDir.y << ", z: " << rayDir.z << std::endl;

    return 0;
}
