A CPU Path tracer.  
![Lucy + Refraction 1000 rays crop](https://github.com/user-attachments/assets/f684c323-53a4-4fee-84b4-b1daf21c2889)

![Colour Blending](https://github.com/user-attachments/assets/52769a1a-4fc1-4780-a5b2-9bcc7b07af49)
  
# Features  
Multiple Importance Sampling (MIS)  
BVH Acceleration  
Multithreading  
PBR Materials  
Meshes and Primitives  
Extended Reinhard Tone Mapping
Controls - WASD and Mouse  
UI to modify render settings  

# To Do  
~~Multiple Importance Sampling (MIS)~~  
Material and Scene Editor  
Russian Roulette Termination  
Denoising  
Next Event Estimation (NEE)  
Restir GI  
Image textures, rougness maps, etc  
Temporal frame reconstruction using motion vectors  
SIMD Instructions to find more paralellism  
GPU Acceleration with Vulkan (maybe)  

# Building  
I use CLion, I'm not quite sure how building works on other systems. A CMake file is included.
It uses SDl2, ImGui and SDL2_Img. I disabled Vulkan for now since I still haven't implemented it.

# Known Bugs  
Probably doesn't work on Linux at the moment  
The Mesh BVH doesn't traverse properly for refractive material  
The Mesh BVH doesn't traverse properly with more than 1 triangle per leaf node  
Rare pitch black pixel, most likely an issue with BRDF and PDF  
