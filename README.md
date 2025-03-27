A CPU Path tracer.  
![Lucy + Refraction 1000 rays crop](https://github.com/user-attachments/assets/f684c323-53a4-4fee-84b4-b1daf21c2889)

![Colour Blending 3100 Rays MIS](https://github.com/user-attachments/assets/19cd37ab-0406-456b-93c2-407642312d4f)

# Features  
Multiple Importance Sampling (MIS)  
ReSTIR (Temporal sampling to come)  
Russian Roulette Termination  
BVH Acceleration  
Multithreading  
PBR Materials  
Meshes and Primitives  
Normal, texture, roughness, metallic, emission maps  
Extended Reinhard Tone Mapping  
Controls - WASD and Mouse  
Scene and Material Editor  

# To Do  
~~Multiple Importance Sampling (MIS)~~  
~~Material and Scene Editor~~  
~~Russian Roulette Termination~~  
Pipeline: Restir + Restir GI + MIS  
Denoising  
~~Texture, roughness, metallic and emission maps~~  
Temporal frame reconstruction using motion vectors  
SIMD Instructions to find more paralellism  
GPU Acceleration with Vulkan (maybe)  

# Building  
I use CLion, I'm not quite sure how building works on other systems. A CMake file is included.
It uses SDl2, ImGui and SDL2_Img. I disabled Vulkan for now since I still haven't implemented it.

# Known Bugs  
Probably doesn't work on Linux at the moment  
Rare pitch black pixel, most likely an issue with BRDF and PDF  
Scale does not working properly for mesh objects  
Denoising is WIP and broken  
Mesh objects do not work as lights for ReSTIR  
Making a Mesh object a light and moving it with ReSTIR will cause a crash  
Rare crash related to index out of bounds  
