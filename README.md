A CPU Path tracer.  
![Lucy + Refraction 1000 rays crop](https://github.com/user-attachments/assets/f684c323-53a4-4fee-84b4-b1daf21c2889)  

![Colour Blending](https://github.com/user-attachments/assets/52769a1a-4fc1-4780-a5b2-9bcc7b07af49)
  
# Features  
Multithreading  
BVH Acceleration  
Meshes  
Cosine Weighted Importance Sampling  
Refraction (Transparency)  
Extended Reinhard Tone Mapping
Depth of Field  
Controls - WASD and Mouse  
UI to modify render settings  

# To Do  
Direct Light Sampling  
Multiple Importance Sampling  
Next Event Estimation (NEE)  
Denoising  
Russian Roulette Termination - (currently ray bounces are a fixed amount)  
Temporal frame reconstruction using motion vectors  
SIMD Instructions to find more paralellism  
GPU Acceleration with Vulkan  
Image textures, rougness maps, etc  

# Improvments
BVH Tree Construction supports both multithreaded and singlethreaded modes. Multithreaded tree construction is only faster for scenes with hundreds of objects, so ST is default.  
![BVH Tree Traversal](https://github.com/user-attachments/assets/e06606b0-830a-4ddc-aae1-cfbb3a9738b1)  
BVH creation is 220x faster and traversal is 34x faster :D  

![Final Comparison](https://github.com/user-attachments/assets/3e9d3384-3d5c-4127-9571-634cd8c5d133)  

2.3x faster than Java for singlethread, and another 8.5x speed up with multithreading. For 8 physical threads that seems reasonable.  
Cinebench has a 9.7x speed up with multithreading so I think thats not bad.  
Edit - I've noticed a scaling issue, at high resolutions the Java version is still faster in singlethread. Not sure what thats about.

# Building  
I use CLion, I'm not quite sure how building works on other systems. A CMake file is included.
It uses SDl2, ImGui and SDL2_Img. I disabled Vulkan for now since I still haven't implemented it.

# Known Bugs  
Probably doesn't work on Linux at the moment
