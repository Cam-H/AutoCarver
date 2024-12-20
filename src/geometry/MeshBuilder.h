//
// Created by Cam on 2024-10-25.
//

#ifndef AUTOCARVER_MESHBUILDER_H
#define AUTOCARVER_MESHBUILDER_H

#include "Mesh.h"

class MeshBuilder {
public:

    static std::shared_ptr<Mesh> plane(float width, const vec3f& origin, const vec3f& normal);
//    static std::shared_ptr<Mesh> plane(float length, float width, const vec3f& origin, const vec3f& normal, const vec3f& ref);
    static std::shared_ptr<Mesh> plane(float length, float width, const vec3f& origin, const vec3f& normal, const vec3f& ref);

    static std::shared_ptr<Mesh> box(float length, float width, float height);

};


#endif //AUTOCARVER_MESHBUILDER_H
