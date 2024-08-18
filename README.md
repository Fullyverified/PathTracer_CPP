Remaking my PathTracer in CPP, now with multithreading.  
  
Building requires the vulkan SDK. In the CMakeLists.txt file find this line:
set(VULKAN_SDK "C:/VulkanSDK/1.3.290.0")  # Adjust the path to your Vulkan SDK version
If you are on Linux use your fancy commands to install the Vulkan SDK and SDL2.
  
I have finished implementing both BVH tree consturction and traversal. Multithreaded tree construction is only faster for scenes with hundreds of objects, so ST is also functional.  
![BVH Tree Traversal](https://github.com/user-attachments/assets/e06606b0-830a-4ddc-aae1-cfbb3a9738b1)  
BVH creation is 220x faster and traversal is 34x faster :D
  
![Final Comparison](https://github.com/user-attachments/assets/3e9d3384-3d5c-4127-9571-634cd8c5d133)  

2.3x faster than Java for singlethread, and another 8.5x speed up with multithreading. For 8 physical threads that seems reasonable.  
Cinebench has a 9.7x speed up with multithreading so I think thats not bad.
