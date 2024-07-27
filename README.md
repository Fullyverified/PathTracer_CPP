Remaking my PathTracer in CPP, hopefully with multithreading.  
I have finished implementing both BVH tree consturction and traversal. Multithreaded tree construction is only faster for scenes with hundreds of objects, so ST is also functional.  
![BVH Tree Traversal](https://github.com/user-attachments/assets/e06606b0-830a-4ddc-aae1-cfbb3a9738b1)  
BVH creation is 220x faster and traversal is 34x faster :D

![image](https://github.com/user-attachments/assets/5cf3b88a-d116-4c41-ab02-28f2350935b9)  
Resolution 3440x3440. 7.1x speed up. For 8 physical threads that seems reasonable.
Cinebench has a 9.7x speed up so I think thats not bad.
