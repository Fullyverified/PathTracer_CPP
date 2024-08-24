Remaking my PathTracer in CPP, now with multithreading.  

![Sphere and Ball 1200x1200 650 Rays](https://github.com/user-attachments/assets/cfc63d02-5af1-46f3-89fb-01018e7ac7d9)

![Sphere and Cube 1200x1200 1000 Rays](https://github.com/user-attachments/assets/8d32a8b7-7f5e-43c1-8ebc-9327847c217a)

# Features  
Multithreading  
BVH Acceleration  
Cosine Weighted Importance Sampling  
Refraction (Transparency)  
Controls - WASD and Mouse  
  input can be locked with Del key for longer renders
Extended Reinhard Tone Mapping

Settings can be altered in the Config.h file. I will add a UI soon.

# Improvments
BVH Tree Construction supports both multithreaded and singlethreaded modes. Multithreaded tree construction is only faster for scenes with hundreds of objects, so ST is default.  
![BVH Tree Traversal](https://github.com/user-attachments/assets/e06606b0-830a-4ddc-aae1-cfbb3a9738b1)  
BVH creation is 220x faster and traversal is 34x faster :D  

![Final Comparison](https://github.com/user-attachments/assets/3e9d3384-3d5c-4127-9571-634cd8c5d133)  

2.3x faster than Java for singlethread, and another 8.5x speed up with multithreading. For 8 physical threads that seems reasonable.  
Cinebench has a 9.7x speed up with multithreading so I think thats not bad.  
Edit - I've noticed a scaling issue, at high resolutions the Java version is still faster in singlethread. Not sure what thats about.

# Building  
Building requires the vulkan SDK to be installed on your system. The project does not currently use Vulkan but I plan to add GPU Acceleration. In the CMakeLists.txt file find this line:  
set(VULKAN_SDK "C:/VulkanSDK/1.3.290.0")  # Adjust the path to your Vulkan SDK version.  
If you are on Linux use your fancy commands to install the Vulkan SDK and SDL2.  
