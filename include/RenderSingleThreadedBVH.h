#ifndef RENDERSINGLETHREADEDBVH_H
#define RENDERSINGLETHREADEDBVH_H

#include <vector>
#include <SceneObject.h>
#include <BVHNode.h>
#include <BoundingBox.h>
#include <memory>

class RenderSingleThreadedBVH {

public:

RenderSingleThreadedBVH();

    void constructBVH(const std::vector<SceneObject*> &sceneObjectsList);

    std::vector<BVHNode*> BVHNodes;


private:
};

#endif //RENDERSINGLETHREADEDBVH_H
